// ═══════════════════════════════════════════════════════════════
// ARIA Hardware Interface — Implementation
// Sim/Hardware switchable ros2_control SystemInterface
//
// SIM mode:  passthrough — Gazebo handles physics via GazeboSimSystem.
//            This class is only reached if it is loaded instead of
//            gazebo_ros2_control; it then simulates perfect tracking.
// REAL mode: bridges ros2_control to the ESP32 micro-ROS firmware
//            (node `aria_esp32`, see esp32_firmware/). It publishes
//            /aria/servo_commands (JointState, degrees) and subscribes
//            /aria/servo_states (JointState, degrees), converting to and
//            from the radians that ros2_control uses. This is an
//            alternative to running arm_control/scripts/servo_sync_node.py;
//            use one or the other, not both.
// ═══════════════════════════════════════════════════════════════
#include "arm_control/aria_hardware_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace arm_control
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegPerRad = 180.0 / kPi;
constexpr double kRadPerDeg = kPi / 180.0;

rclcpp::Logger logger()
{
  return rclcpp::get_logger("AriaHardwareInterface");
}
}  // namespace

// ═══════════════════════════════════════════════════════════════
// on_init — Load parameters from URDF <ros2_control> tag
// ═══════════════════════════════════════════════════════════════
hardware_interface::CallbackReturn AriaHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Parse parameters from the URDF hardware tag.
  if (info.hardware_parameters.count("use_sim")) {
    use_sim_ = (info.hardware_parameters.at("use_sim") == "true");
  }
  if (info.hardware_parameters.count("command_topic")) {
    command_topic_ = info.hardware_parameters.at("command_topic");
  }
  if (info.hardware_parameters.count("state_topic")) {
    state_topic_ = info.hardware_parameters.at("state_topic");
  }
  if (info.hardware_parameters.count("teach_topic")) {
    teach_topic_ = info.hardware_parameters.at("teach_topic");
  }
  if (info.hardware_parameters.count("state_timeout_s")) {
    state_timeout_s_ = std::stod(info.hardware_parameters.at("state_timeout_s"));
  }

  // Validate joint configuration.
  if (info.joints.size() != NUM_JOINTS) {
    RCLCPP_ERROR(
      logger(), "Expected %zu joints, got %zu", NUM_JOINTS, info.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const auto & joint : info.joints) {
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_ERROR(
        logger(), "Joint '%s' must have exactly 1 command interface, got %zu",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.command_interfaces[0].name !=
      hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(
        logger(), "Joint '%s' command interface must be 'position'",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces.size() != 2) {
      RCLCPP_ERROR(
        logger(), "Joint '%s' must have exactly 2 state interfaces, got %zu",
        joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  hw_joint_names_.clear();
  for (const auto & joint : info.joints) {
    hw_joint_names_.push_back(joint.name);
  }

  hw_positions_.assign(NUM_JOINTS, 0.0);
  hw_velocities_.assign(NUM_JOINTS, 0.0);
  hw_commands_.assign(NUM_JOINTS, 0.0);
  latest_positions_.assign(NUM_JOINTS, 0.0);
  latest_velocities_.assign(NUM_JOINTS, std::nan(""));

  RCLCPP_INFO(
    logger(), "ARIA Hardware Interface: mode = %s",
    use_sim_ ? "SIM" : "HARDWARE");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════
// on_configure — Set up communication backend
// ═══════════════════════════════════════════════════════════════
hardware_interface::CallbackReturn AriaHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (use_sim_) {
    RCLCPP_INFO(
      logger(), "Sim mode: Gazebo (gz_ros2_control) drives the joints; "
      "no ESP32 bridge created");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  RCLCPP_INFO(
    logger(), "Hardware mode: bridging to ESP32 micro-ROS node via "
    "'%s' (cmd) / '%s' (state)",
    command_topic_.c_str(), state_topic_.c_str());

  if (!rclcpp::ok()) {
    RCLCPP_ERROR(
      logger(), "rclcpp is not initialised; cannot create the ESP32 bridge");
    return hardware_interface::CallbackReturn::ERROR;
  }

  try {
    node_ = rclcpp::Node::make_shared("aria_hardware_interface");
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    servo_cmd_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
      command_topic_, qos);
    teach_mode_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
      teach_topic_, qos);
    servo_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      state_topic_, qos,
      [this](sensor_msgs::msg::JointState::SharedPtr msg) {
        this->state_callback(msg);
      });
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger(), "Failed to create ESP32 bridge: %s", e.what());
    node_.reset();
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════
// on_activate — Enable hardware
// ═══════════════════════════════════════════════════════════════
hardware_interface::CallbackReturn AriaHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    hw_commands_[i] = hw_positions_[i];
  }

  if (use_sim_) {
    RCLCPP_INFO(logger(), "Sim mode activated: controllers handled by launch");
  } else if (teach_mode_pub_) {
    // Engage servo torque (teach_mode=false re-attaches the servos).
    std_msgs::msg::Bool engage;
    engage.data = false;
    teach_mode_pub_->publish(engage);
    RCLCPP_INFO(logger(), "Hardware mode activated: servos engaged");
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════
// on_deactivate — Disable hardware safely
// ═══════════════════════════════════════════════════════════════
hardware_interface::CallbackReturn AriaHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (use_sim_) {
    RCLCPP_INFO(logger(), "Sim mode deactivated");
  } else if (teach_mode_pub_) {
    // Relax the servos so the arm can be moved by hand (safe power-off).
    std_msgs::msg::Bool relax;
    relax.data = true;
    teach_mode_pub_->publish(relax);
    RCLCPP_INFO(logger(), "Hardware mode: servos relaxed (safe power-off)");
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════
// on_cleanup — Release resources
// ═══════════════════════════════════════════════════════════════
hardware_interface::CallbackReturn AriaHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  servo_state_sub_.reset();
  servo_cmd_pub_.reset();
  teach_mode_pub_.reset();
  node_.reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════
// export_state_interfaces — position + velocity per joint
// ═══════════════════════════════════════════════════════════════
std::vector<hardware_interface::StateInterface>
AriaHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    state_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_positions_[i]);
    state_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_VELOCITY,
      &hw_velocities_[i]);
  }
  return state_interfaces;
}

// ═══════════════════════════════════════════════════════════════
// export_command_interfaces — position command per joint
// ═══════════════════════════════════════════════════════════════
std::vector<hardware_interface::CommandInterface>
AriaHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    command_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_commands_[i]);
  }
  return command_interfaces;
}

// ═══════════════════════════════════════════════════════════════
// read — Update joint states from simulation or hardware
// ═══════════════════════════════════════════════════════════════
hardware_interface::return_type AriaHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (use_sim_) {
    // In sim mode, GazeboSimSystem handles read() directly. This path is
    // only reached if this plugin is used instead; simulate perfect
    // tracking so controllers still behave.
    for (size_t i = 0; i < NUM_JOINTS; ++i) {
      double prev_pos = hw_positions_[i];
      hw_positions_[i] = hw_commands_[i];
      if (period.seconds() > 0.0) {
        hw_velocities_[i] = (hw_positions_[i] - prev_pos) / period.seconds();
      }
    }
    return hardware_interface::return_type::OK;
  }

  if (!node_) {
    return hardware_interface::return_type::OK;
  }

  // Pump the bridge callbacks (the controller_manager does not spin it).
  rclcpp::spin_some(node_);

  std::lock_guard<std::mutex> lock(state_mutex_);
  const bool fresh = state_received_ &&
    (std::chrono::steady_clock::now() - last_state_time_) <=
    std::chrono::duration<double>(state_timeout_s_);

  if (!fresh) {
    RCLCPP_WARN_THROTTLE(
      logger(), *node_->get_clock(), 2000,
      "No fresh /aria/servo_states feedback from the ESP32 "
      "(holding last known joint positions). Is the micro-ROS agent up?");
    return hardware_interface::return_type::OK;
  }

  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    double prev_pos = hw_positions_[i];
    hw_positions_[i] = latest_positions_[i];

    double v = latest_velocities_[i];
    if (!std::isfinite(v) && period.seconds() > 0.0) {
      v = (hw_positions_[i] - prev_pos) / period.seconds();
    }
    hw_velocities_[i] = std::isfinite(v) ? v : 0.0;
  }

  return hardware_interface::return_type::OK;
}

// ═══════════════════════════════════════════════════════════════
// write — Send commands to simulation or hardware
// ═══════════════════════════════════════════════════════════════
hardware_interface::return_type AriaHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (use_sim_) {
    // GazeboSimSystem reads hw_commands_ via the command interfaces.
    return hardware_interface::return_type::OK;
  }

  if (!servo_cmd_pub_) {
    return hardware_interface::return_type::OK;
  }

  // hw_commands_ are radians; the firmware expects degrees in the
  // canonical joint order (it indexes by position, ignoring names).
  sensor_msgs::msg::JointState cmd;
  cmd.header.stamp = node_->now();
  cmd.name.reserve(NUM_JOINTS);
  cmd.position.reserve(NUM_JOINTS);

  for (const auto & name : JOINT_NAMES) {
    const int idx = find_joint(name);
    cmd.name.push_back(name);
    if (idx >= 0) {
      cmd.position.push_back(hw_commands_[idx] * kDegPerRad);
    } else {
      cmd.position.push_back(std::nan(""));
    }
  }

  servo_cmd_pub_->publish(cmd);
  return hardware_interface::return_type::OK;
}

// ═══════════════════════════════════════════════════════════════
// find_joint — index of a joint name in the hardware info
// ═══════════════════════════════════════════════════════════════
int AriaHardwareInterface::find_joint(const std::string & name) const
{
  const auto it = std::find(hw_joint_names_.begin(), hw_joint_names_.end(), name);
  if (it == hw_joint_names_.end()) {
    return -1;
  }
  return static_cast<int>(std::distance(hw_joint_names_.begin(), it));
}

// ═══════════════════════════════════════════════════════════════
// state_callback — store ESP32 servo feedback (degrees → radians)
// ═══════════════════════════════════════════════════════════════
void AriaHardwareInterface::state_callback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  const size_t n = msg->position.size();
  if (msg->name.empty()) {
    // Unnamed feedback: assume canonical order.
    for (size_t i = 0; i < NUM_JOINTS && i < n; ++i) {
      latest_positions_[i] = msg->position[i] * kRadPerDeg;
    }
  } else {
    for (size_t k = 0; k < msg->name.size() && k < n; ++k) {
      const int idx = find_joint(msg->name[k]);
      if (idx >= 0) {
        latest_positions_[idx] = msg->position[k] * kRadPerDeg;
      }
    }
  }

  // Velocity is optional; leave NaN when the firmware does not report it.
  if (msg->velocity.size() == msg->position.size()) {
    if (msg->name.empty()) {
      for (size_t i = 0; i < NUM_JOINTS && i < msg->velocity.size(); ++i) {
        latest_velocities_[i] = msg->velocity[i] * kRadPerDeg;
      }
    } else {
      for (size_t k = 0; k < msg->name.size() && k < msg->velocity.size(); ++k) {
        const int idx = find_joint(msg->name[k]);
        if (idx >= 0) {
          latest_velocities_[idx] = msg->velocity[k] * kRadPerDeg;
        }
      }
    }
  } else {
    std::fill(latest_velocities_.begin(), latest_velocities_.end(), std::nan(""));
  }

  state_received_ = true;
  last_state_time_ = std::chrono::steady_clock::now();
}

}  // namespace arm_control

// ── Register plugin with pluginlib ────────────────────────────
PLUGINLIB_EXPORT_CLASS(
  arm_control::AriaHardwareInterface,
  hardware_interface::SystemInterface)
