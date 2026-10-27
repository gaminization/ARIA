// ═══════════════════════════════════════════════════════════════
// ARIA Industrial Conveyor Belt Gazebo Plugin
// High-Fidelity Physics Engine:
// 1. Joint velocity animation on prismatic rubber belt.
// 2. Continuous surface propulsion for workpiece transport with zero jitter.
// 3. Precision docking against mechanical end-stopper.
// 4. Optical part presence detection and ROS 2 service/topic interfaces.
// ═══════════════════════════════════════════════════════════════

#include "arm_control/conveyor_belt_plugin.hpp"

#include <gazebo/physics/World.hh>
#include <gazebo/physics/Link.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <algorithm>

namespace gazebo_ros
{

AriaConveyorPlugin::AriaConveyorPlugin() = default;

AriaConveyorPlugin::~AriaConveyorPlugin()
{
  update_connection_.reset();
}

void AriaConveyorPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  model_ = _model;
  world_ = model_->GetWorld();
  ros_node_ = gazebo_ros::Node::Get(_sdf);

  if (_sdf->HasElement("joint_name")) {
    joint_name_ = _sdf->Get<std::string>("joint_name");
  }

  belt_joint_ = model_->GetJoint(joint_name_);
  if (!belt_joint_) {
    RCLCPP_ERROR(
      ros_node_->get_logger(),
      "[AriaConveyorPlugin] Belt joint '%s' not found in model '%s'!",
      joint_name_.c_str(), model_->GetName().c_str());
    return;
  }

  if (_sdf->HasElement("max_velocity")) {
    max_velocity_ = _sdf->Get<double>("max_velocity");
  }

  if (_sdf->HasElement("limit")) {
    limit_ = _sdf->Get<double>("limit");
  } else {
    limit_ = belt_joint_->UpperLimit();
    if (limit_ <= 0.0 || limit_ > 10.0) {
      limit_ = 0.02;  // 20mm stroke
    }
  }

  double publish_rate = 20.0;
  if (_sdf->HasElement("publish_rate")) {
    publish_rate = _sdf->Get<double>("publish_rate");
  }
  update_interval_ns_ = static_cast<int>((1.0 / publish_rate) * 1e9);

  // Initial power
  if (_sdf->HasElement("initial_power")) {
    power_pct_ = _sdf->Get<double>("initial_power");
    power_pct_ = std::clamp(power_pct_, 0.0, 100.0);
    belt_velocity_ = max_velocity_ * (power_pct_ / 100.0);
  }

  // ROS 2 Service: /aria/conveyor/set_power
  power_service_ = ros_node_->create_service<arm_interfaces::srv::SetConveyorPower>(
    "/aria/conveyor/set_power",
    std::bind(&AriaConveyorPlugin::SetPowerService, this, std::placeholders::_1, std::placeholders::_2));

  // ROS 2 Subscription: /aria/conveyor/cmd_vel
  cmd_vel_sub_ = ros_node_->create_subscription<geometry_msgs::msg::Twist>(
    "/aria/conveyor/cmd_vel", 10,
    std::bind(&AriaConveyorPlugin::CmdVelCallback, this, std::placeholders::_1));

  // ROS 2 Publishers
  speed_pub_ = ros_node_->create_publisher<std_msgs::msg::Float64>("/aria/conveyor/speed", 10);
  running_pub_ = ros_node_->create_publisher<std_msgs::msg::Bool>("/aria/conveyor/running", 10);
  part_presence_pub_ = ros_node_->create_publisher<std_msgs::msg::Bool>("/aria/conveyor/part_present", 10);

  last_publish_time_ = ros_node_->get_clock()->now();

  // Connect physics update loop
  update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&AriaConveyorPlugin::OnUpdate, this));

  // Optional rotating drum joints
  head_roller_joint_ = model_->GetJoint("head_roller_joint");
  tail_roller_joint_ = model_->GetJoint("tail_roller_joint");

  RCLCPP_INFO(
    ros_node_->get_logger(),
    "[AriaConveyorPlugin] Conveyor '%s' online (joint: %s, max_vel: %.2f m/s, limit: %.3f m)",
    model_->GetName().c_str(), joint_name_.c_str(), max_velocity_, limit_);
}

void AriaConveyorPlugin::OnUpdate()
{
  if (!belt_joint_) {
    return;
  }

  // 1. Joint velocity animation for rubber belt visual surface
  belt_joint_->SetVelocity(0, belt_velocity_);

  // Smooth stroboscopic position recycling: wrap by exact slat pitch without velocity jerk
  double current_pos = belt_joint_->Position(0);
  if (current_pos >= limit_ || current_pos < 0.0) {
    belt_joint_->SetPosition(0, current_pos >= limit_ ? (current_pos - limit_) : 0.0);
  }

  // 2. Continuous rotating drum animation for head and tail pulleys
  if (head_roller_joint_) {
    head_roller_joint_->SetVelocity(0, belt_velocity_ / roller_radius_);
  }
  if (tail_roller_joint_) {
    tail_roller_joint_->SetVelocity(0, belt_velocity_ / roller_radius_);
  }

  // 2. High-precision object surface propulsion & queue accumulation on the conveyor deck
  bool part_at_pick_station = false;
  if (world_) {
    struct ActivePart {
      double py;
      gazebo::physics::LinkPtr link;
    };
    std::vector<ActivePart> active_parts;

    const auto & models = world_->Models();
    for (const auto & m : models) {
      if (!m || m == model_) continue;
      const std::string & name = m->GetName();

      // Identify dynamic objects (workpieces, small blocks)
      if (name.find("workpiece") != std::string::npos ||
          name.find("part") != std::string::npos ||
          name.find("cube") != std::string::npos ||
          name.find("box") != std::string::npos) {

        for (const auto & link : m->GetLinks()) {
          if (!link) continue;
          ignition::math::Pose3d pose = link->WorldPose();
          double px = pose.Pos().X();
          double py = pose.Pos().Y();
          double pz = pose.Pos().Z();

          // Check if workpiece is resting on the conveyor deck
          if (px >= min_x_ && px <= max_x_ &&
              py >= min_y_ && py <= max_y_ &&
              pz >= min_z_ && pz <= max_z_) {

            active_parts.push_back({py, link});

            // Check if workpiece has arrived at the pick stopper window
            if (py <= (stop_y_ + 0.025)) {
              part_at_pick_station = true;
            }
          }
        }
      }
    }

    if (!active_parts.empty()) {
      // Sort workpieces by Y ascending (from front/stopper to rear/infeed)
      std::sort(active_parts.begin(), active_parts.end(),
        [](const ActivePart & a, const ActivePart & b) {
          return a.py < b.py;
        });
    }

    // The velocity override below exists only to carry parts along a *running*
    // belt.  It zeroes each part's vertical velocity and angular velocity, so
    // applying it while the belt is stopped pins the workpieces to the deck and
    // makes them physically impossible to lift (a 50 mm pick would stall at the
    // top of the detection window, z = 0.680 m).  Gate it on belt motion so a
    // stopped belt leaves the parts as free rigid bodies for manipulation.
    const bool belt_running = std::abs(belt_velocity_) > 1e-4;

    const double sim_time = world_->SimTime().Double();
    if (sim_time - last_diagnostic_time_ >= 1.0) {
      const double lead_y = active_parts.empty() ? 0.0 : active_parts[0].py;
      const double next_y = active_parts.size() < 2 ? 0.0 : active_parts[1].py;
      RCLCPP_INFO(
        ros_node_->get_logger(),
        "[AriaConveyorPlugin] power=%.1f%% active_parts=%zu lead_y=%.3f next_y=%.3f",
        power_pct_, active_parts.size(), lead_y, next_y);
      last_diagnostic_time_ = sim_time;
    }

    if (!active_parts.empty() && belt_running) {
      for (size_t i = 0; i < active_parts.size(); ++i) {
        auto & part = active_parts[i];
        // The lead part stops at stop_y_ against the stopper.
        // Each successive queued part stops neatly behind the preceding part (30mm cube + 15mm spacing)
        double target_stop_y = stop_y_ + i * 0.045;
        if (i > 0) {
          double prev_py = active_parts[i - 1].py;
          target_stop_y = std::max(target_stop_y, prev_py + 0.045);
        }

        // Precision centerline guide (centering on conveyor track x = 0.200)
        double px = part.link->WorldPose().Pos().X();
        double x_err = 0.200 - px;
        double vx = std::clamp(x_err * 3.0, -0.06, 0.06);
        const double target_y_velocity =
          std::abs(belt_velocity_) > 1e-4 && part.py > target_stop_y ?
          -belt_velocity_ : 0.0;
        const ignition::math::Vector3d cur_vel = part.link->WorldLinearVel();
        const double target_y_acceleration = std::clamp(
          (target_y_velocity - cur_vel.Y()) * 20.0, -2.0, 2.0);
        const double mass = part.link->GetInertial()->Mass();
        part.link->AddForce(
          ignition::math::Vector3d(0.0, mass * target_y_acceleration, 0.0));
        const double target_x_velocity =
          std::abs(belt_velocity_) > 1e-4 ? vx : 0.0;
        const double z_vel = std::min(0.0, cur_vel.Z());
        part.link->SetLinearVel(
          ignition::math::Vector3d(target_x_velocity, cur_vel.Y(), z_vel));
        part.link->SetAngularVel(ignition::math::Vector3d(0.0, 0.0, 0.0));
      }
    }
  }

  // 3. Periodic ROS 2 status broadcast (20 Hz)
  rclcpp::Time now = ros_node_->get_clock()->now();
  if ((now - last_publish_time_).nanoseconds() >= update_interval_ns_) {
    std_msgs::msg::Float64 speed_msg;
    speed_msg.data = belt_velocity_;
    speed_pub_->publish(speed_msg);

    std_msgs::msg::Bool running_msg;
    running_msg.data = (std::abs(belt_velocity_) > 1e-4);
    running_pub_->publish(running_msg);

    std_msgs::msg::Bool presence_msg;
    presence_msg.data = part_at_pick_station;
    part_presence_pub_->publish(presence_msg);

    last_publish_time_ = now;
  }
}

void AriaConveyorPlugin::SetPowerService(
  const std::shared_ptr<arm_interfaces::srv::SetConveyorPower::Request> req,
  std::shared_ptr<arm_interfaces::srv::SetConveyorPower::Response> res)
{
  if (req->power < 0.0 || req->power > 100.0) {
    res->success = false;
    res->message = "Power value must be between 0.0 and 100.0 percent.";
    RCLCPP_WARN(ros_node_->get_logger(), "[AriaConveyorPlugin] %s", res->message.c_str());
    return;
  }

  power_pct_ = req->power;
  belt_velocity_ = max_velocity_ * (power_pct_ / 100.0);

  res->success = true;
  res->message = "Conveyor power set to " + std::to_string(power_pct_) + "% (velocity: " +
                 std::to_string(belt_velocity_) + " m/s).";
  RCLCPP_INFO(ros_node_->get_logger(), "[AriaConveyorPlugin] %s", res->message.c_str());
}

void AriaConveyorPlugin::CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  double cmd = (std::abs(msg->linear.x) > std::abs(msg->linear.y)) ? msg->linear.x : msg->linear.y;
  cmd = std::clamp(cmd, -max_velocity_, max_velocity_);
  belt_velocity_ = cmd;
  power_pct_ = (belt_velocity_ / max_velocity_) * 100.0;
}

GZ_REGISTER_MODEL_PLUGIN(AriaConveyorPlugin)

}  // namespace gazebo_ros
