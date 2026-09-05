#ifndef ARM_CONTROL__GRIPPER_PLUGIN_HPP_
#define ARM_CONTROL__GRIPPER_PLUGIN_HPP_

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <memory>
#include <string>

namespace gazebo_ros
{

class AriaGripperPlugin : public gazebo::ModelPlugin
{
public:
  AriaGripperPlugin();
  virtual ~AriaGripperPlugin();

  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

private:
  void AttachService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  void DetachService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  gazebo::physics::ModelPtr model_;
  gazebo::physics::WorldPtr world_;
  gazebo::physics::LinkPtr palm_link_;
  gazebo::physics::JointPtr grasp_joint_;
  std::string attached_model_name_;

  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr attach_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr detach_service_;

  std::string palm_link_name_{"wrist_link"};
  double max_attach_dist_{0.14};  // 140 mm search radius from wrist
};

}  // namespace gazebo_ros

#endif  // ARM_CONTROL__GRIPPER_PLUGIN_HPP_
