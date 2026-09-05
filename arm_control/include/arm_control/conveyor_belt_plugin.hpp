#ifndef ARM_CONTROL__CONVEYOR_BELT_PLUGIN_HPP_
#define ARM_CONTROL__CONVEYOR_BELT_PLUGIN_HPP_

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <arm_interfaces/srv/set_conveyor_power.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>

#include <memory>
#include <string>

namespace gazebo_ros
{

class AriaConveyorPlugin : public gazebo::ModelPlugin
{
public:
  AriaConveyorPlugin();
  virtual ~AriaConveyorPlugin();

  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

private:
  void OnUpdate();
  void SetPowerService(
    const std::shared_ptr<arm_interfaces::srv::SetConveyorPower::Request> req,
    std::shared_ptr<arm_interfaces::srv::SetConveyorPower::Response> res);
  void CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  gazebo_ros::Node::SharedPtr ros_node_;
  gazebo::physics::WorldPtr world_;
  gazebo::physics::ModelPtr model_;
  gazebo::physics::JointPtr belt_joint_;
  gazebo::event::ConnectionPtr update_connection_;

  double max_velocity_{0.25};
  double belt_velocity_{0.0};
  double power_pct_{0.0};
  double limit_{0.060};
  std::string joint_name_{"belt_joint"};
  gazebo::physics::JointPtr head_roller_joint_;
  gazebo::physics::JointPtr tail_roller_joint_;
  double roller_radius_{0.014};

  // Conveyor surface bounding box for smooth contact propulsion
  double min_x_{0.09};
  double max_x_{0.29};
  double min_y_{0.045};
  double max_y_{0.80};
  double min_z_{0.620};
  double max_z_{0.680};
  double stop_y_{0.070};

  rclcpp::Service<arm_interfaces::srv::SetConveyorPower>::SharedPtr power_service_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr running_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr part_presence_pub_;

  rclcpp::Time last_publish_time_;
  int update_interval_ns_{50000000};  // 20 Hz
};

}  // namespace gazebo_ros

#endif  // ARM_CONTROL__CONVEYOR_BELT_PLUGIN_HPP_
