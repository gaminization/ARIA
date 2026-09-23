// ═══════════════════════════════════════════════════════════════
// ARIA Industrial Dynamic Grasping Plugin for Gazebo Classic 11
// Provides flawless physical grasp attachment and release
// without penalty contact noise or slippage.
// ═══════════════════════════════════════════════════════════════

#include "arm_control/gripper_plugin.hpp"

#include <gazebo/physics/World.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo_ros
{

AriaGripperPlugin::AriaGripperPlugin() = default;

AriaGripperPlugin::~AriaGripperPlugin()
{
  if (grasp_joint_) {
    grasp_joint_->Detach();
    grasp_joint_.reset();
  }
}

void AriaGripperPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  model_ = _model;
  world_ = model_->GetWorld();
  ros_node_ = gazebo_ros::Node::Get(_sdf);

  if (_sdf->HasElement("palm_link")) {
    palm_link_name_ = _sdf->Get<std::string>("palm_link");
  }
  palm_link_ = model_->GetLink(palm_link_name_);

  if (!palm_link_) {
    RCLCPP_WARN(
      ros_node_->get_logger(),
      "[AriaGripperPlugin] Specified palm link '%s' not found, falling back to wrist_link",
      palm_link_name_.c_str());
    palm_link_ = model_->GetLink("wrist_link");
  }

  if (_sdf->HasElement("max_attach_dist")) {
    max_attach_dist_ = _sdf->Get<double>("max_attach_dist");
  }

  // Joints for dual-finger symmetric actuation
  gripper_joint_ = model_->GetJoint("gripper_joint");
  mimic_joint_ = model_->GetJoint("finger_mimic_joint");

  // Hook world update for continuous mimic joint synchronization
  update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&AriaGripperPlugin::OnUpdate, this));

  // ROS 2 Services
  attach_service_ = ros_node_->create_service<std_srvs::srv::Trigger>(
    "/aria/gripper/attach",
    std::bind(&AriaGripperPlugin::AttachService, this, std::placeholders::_1, std::placeholders::_2));

  detach_service_ = ros_node_->create_service<std_srvs::srv::Trigger>(
    "/aria/gripper/detach",
    std::bind(&AriaGripperPlugin::DetachService, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(
    ros_node_->get_logger(),
    "[AriaGripperPlugin] Industrial Grasp Plugin online (palm link: %s, max_dist: %.3f m, mimic: %s)",
    palm_link_ ? palm_link_->GetName().c_str() : "NULL", max_attach_dist_,
    mimic_joint_ ? "active" : "missing");
}

void AriaGripperPlugin::OnUpdate()
{
  if (gripper_joint_ && mimic_joint_) {
    double pos = gripper_joint_->Position(0);
    mimic_joint_->SetPosition(0, pos);
  }
}

void AriaGripperPlugin::AttachService(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  if (!world_ || !palm_link_) {
    res->success = false;
    res->message = "World or palm link invalid";
    return;
  }

  if (grasp_joint_) {
    res->success = true;
    res->message = "Already holding object: " + attached_model_name_;
    return;
  }

  ignition::math::Vector3d palm_pos = palm_link_->WorldPose().Pos();
  RCLCPP_INFO(
    ros_node_->get_logger(),
    "[AriaGripperPlugin] Attach request: palm link '%s' pos=(%.3f, %.3f, %.3f), max_dist=%.3f",
    palm_link_ ? palm_link_->GetName().c_str() : "NULL",
    palm_pos.X(), palm_pos.Y(), palm_pos.Z(), max_attach_dist_);

  // Find nearest dynamic workpiece strictly within physical grasp envelope (default 14cm from wrist knuckle)
  double min_dist = (max_attach_dist_ > 0.0) ? max_attach_dist_ : 0.14;
  gazebo::physics::ModelPtr best_model = nullptr;
  gazebo::physics::LinkPtr best_link = nullptr;

  for (const auto & m : world_->Models()) {
    if (!m || m == model_ || m->IsStatic()) continue;
    const std::string & name = m->GetName();
    if (name.find("table") != std::string::npos ||
        name.find("ground") != std::string::npos ||
        name.find("camera") != std::string::npos ||
        name.find("inspection_rig") != std::string::npos ||
        name.find("light") != std::string::npos ||
        name.find("plane") != std::string::npos) {
      continue;
    }

    for (const auto & l : m->GetLinks()) {
      if (!l) continue;
      double d = (l->WorldPose().Pos() - palm_pos).Length();
      RCLCPP_INFO(
        ros_node_->get_logger(),
        "[AriaGripperPlugin] Candidate model '%s' link '%s' pos=(%.3f, %.3f, %.3f), dist=%.3f",
        m->GetName().c_str(), l->GetName().c_str(),
        l->WorldPose().Pos().X(), l->WorldPose().Pos().Y(), l->WorldPose().Pos().Z(), d);
      if (d < min_dist) {
        min_dist = d;
        best_model = m;
        best_link = l;
      }
    }
  }

  if (!best_model || !best_link) {
    res->success = false;
    res->message = "No workpiece detected within gripper envelope";
    return;
  }

  // Create ODE dynamic fixed joint
  grasp_joint_ = world_->Physics()->CreateJoint("fixed", model_);
  grasp_joint_->SetName(model_->GetName() + "_grasp_" + best_model->GetName());
  grasp_joint_->Attach(palm_link_, best_link);
  grasp_joint_->Load(palm_link_, best_link, ignition::math::Pose3d());
  grasp_joint_->Init();

  attached_model_name_ = best_model->GetName();
  res->success = true;
  res->message = "Securely attached workpiece: " + attached_model_name_ + " (dist: " + std::to_string(min_dist) + " m)";

  RCLCPP_INFO(
    ros_node_->get_logger(),
    "[AriaGripperPlugin] ⚡ Grasp locked on '%s' (dist: %.3f m)",
    attached_model_name_.c_str(), min_dist);
}

void AriaGripperPlugin::DetachService(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  if (!grasp_joint_) {
    res->success = true;
    res->message = "No object currently held";
    return;
  }

  grasp_joint_->Detach();
  grasp_joint_.reset();

  RCLCPP_INFO(
    ros_node_->get_logger(),
    "[AriaGripperPlugin] ⚡ Grasp released '%s'",
    attached_model_name_.c_str());

  res->success = true;
  res->message = "Released workpiece: " + attached_model_name_;
  attached_model_name_.clear();
}

GZ_REGISTER_MODEL_PLUGIN(AriaGripperPlugin)

}  // namespace gazebo_ros
