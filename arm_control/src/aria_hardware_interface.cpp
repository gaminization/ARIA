// ═══════════════════════════════════════════════════════════════
// ARIA Hardware Interface — Implementation
// Sim/Hardware switchable ros2_control SystemInterface
//
// SIM mode:  passthrough — Gazebo handles physics via GazeboSimSystem.
//            This class is used as a fallback/placeholder; in practice,
//            gz_ros2_control loads GazeboSimSystem directly.
// REAL mode: communicates with ESP32 via serial JSON protocol (Stage 4).
// ═══════════════════════════════════════════════════════════════
#include "arm_control/aria_hardware_interface.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace arm_control
{

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

  // Parse parameters from URDF hardware tag
  if (info.hardware_parameters.count("use_sim")) {
    use_sim_ = (info.hardware_parameters.at("use_sim") == "true");
  }
  if (info.hardware_parameters.count("serial_port")) {
    serial_port_ = info.hardware_parameters.at("serial_port");
  }
  if (info.hardware_parameters.count("baud_rate")) {
    baud_rate_ = std::stoi(info.hardware_parameters.at("baud_rate"));
  }

  // Validate joint configuration
  if (info.joints.size() != NUM_JOINTS) {
    RCLCPP_ERROR(
      rclcpp::get_logger("AriaHardwareInterface"),
      "Expected %zu joints, got %zu", NUM_JOINTS, info.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Validate interfaces for each joint
  for (const auto & joint : info.joints) {
    // Exactly 1 command interface (position)
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_ERROR(
        rclcpp::get_logger("AriaHardwareInterface"),
        "Joint '%s' must have exactly 1 command interface, got %zu",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.command_interfaces[0].name !=
      hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("AriaHardwareInterface"),
        "Joint '%s' command interface must be 'position'",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    // Exactly 2 state interfaces (position, velocity)
    if (joint.state_interfaces.size() != 2) {
      RCLCPP_ERROR(
        rclcpp::get_logger("AriaHardwareInterface"),
        "Joint '%s' must have exactly 2 state interfaces, got %zu",
        joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Initialize state/command vectors
  hw_positions_.resize(NUM_JOINTS, 0.0);
  hw_velocities_.resize(NUM_JOINTS, 0.0);
  hw_commands_.resize(NUM_JOINTS, 0.0);

  RCLCPP_INFO(
    rclcpp::get_logger("AriaHardwareInterface"),
    "ARIA Hardware Interface: mode = %s",
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
      rclcpp::get_logger("AriaHardwareInterface"),
      "Sim mode: using Gazebo joint states (no serial connection)");
  } else {
    RCLCPP_INFO(
      rclcpp::get_logger("AriaHardwareInterface"),
      "Hardware mode: opening serial port %s at %d baud",
      serial_port_.c_str(), baud_rate_);

    if (!open_serial_port()) {
      RCLCPP_ERROR(
        rclcpp::get_logger("AriaHardwareInterface"),
        "Failed to open serial port %s", serial_port_.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Ping ESP32 to verify connection
    if (!send_serial_command("{\"cmd\": \"ping\"}")) {
      RCLCPP_ERROR(
        rclcpp::get_logger("AriaHardwareInterface"),
        "ESP32 did not respond to ping");
      close_serial_port();
      return hardware_interface::CallbackReturn::ERROR;
    }

    std::string response = read_serial_response();
    if (response.find("pong") == std::string::npos) {
      RCLCPP_ERROR(
        rclcpp::get_logger("AriaHardwareInterface"),
        "ESP32 ping response invalid: %s", response.c_str());
      close_serial_port();
      return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(
      rclcpp::get_logger("AriaHardwareInterface"),
      "ESP32 connected successfully");
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════
// on_activate — Enable hardware
// ═══════════════════════════════════════════════════════════════
hardware_interface::CallbackReturn AriaHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Initialize commands to current positions
  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    hw_commands_[i] = hw_positions_[i];
  }

  if (use_sim_) {
    RCLCPP_INFO(
      rclcpp::get_logger("AriaHardwareInterface"),
      "Sim mode activated: controllers handled by launch system");
  } else {
    // Enable servo torque and move to home position
    RCLCPP_INFO(
      rclcpp::get_logger("AriaHardwareInterface"),
      "Hardware mode activated: enabling servo torque");
    send_serial_command("{\"cmd\": \"enable_torque\"}");
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
    RCLCPP_INFO(
      rclcpp::get_logger("AriaHardwareInterface"),
      "Sim mode deactivated");
  } else {
    // Disable servo torque for safe power-off
    RCLCPP_INFO(
      rclcpp::get_logger("AriaHardwareInterface"),
      "Hardware mode: disabling servo torque (safe power-off)");
    send_serial_command("{\"cmd\": \"disable_torque\"}");
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════
// on_cleanup — Release resources
// ═══════════════════════════════════════════════════════════════
hardware_interface::CallbackReturn AriaHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!use_sim_ && serial_fd_ >= 0) {
    close_serial_port();
  }
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
    // In sim mode, GazeboSimSystem handles read() directly.
    // This code path is only reached if using this plugin instead of
    // GazeboSimSystem. In that case, simulate perfect tracking:
    for (size_t i = 0; i < NUM_JOINTS; ++i) {
      double prev_pos = hw_positions_[i];
      hw_positions_[i] = hw_commands_[i];  // Perfect tracking in sim
      if (period.seconds() > 0.0) {
        hw_velocities_[i] = (hw_positions_[i] - prev_pos) / period.seconds();
      }
    }
  } else {
    // HARDWARE MODE: Read from ESP32 serial
    // Parse JSON: {"pos": [j1..j6], "vel": [v1..v6], "ts": timestamp}
    // Implementation in Stage 4
    std::string response = read_serial_response();
    if (!response.empty()) {
      // TODO(Stage 4): Parse JSON response into hw_positions_, hw_velocities_
      RCLCPP_DEBUG(
        rclcpp::get_logger("AriaHardwareInterface"),
        "Serial read: %s", response.c_str());
    }
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
    // In sim mode, GazeboSimSystem handles write() directly.
    // No additional action needed; commands are already in hw_commands_
    // which GazeboSimSystem reads via the command interfaces.
  } else {
    // HARDWARE MODE: Send to ESP32
    // Format: {"cmd": [j1..j6], "gripper": gval, "ts": timestamp}
    // Implementation in Stage 4
    char cmd_buf[256];
    snprintf(cmd_buf, sizeof(cmd_buf),
      "{\"cmd\": [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]}",
      hw_commands_[0], hw_commands_[1], hw_commands_[2],
      hw_commands_[3], hw_commands_[4], hw_commands_[5]);
    send_serial_command(std::string(cmd_buf));
  }

  return hardware_interface::return_type::OK;
}

// ═══════════════════════════════════════════════════════════════
// Serial port helpers — Skeleton for Stage 4 hardware integration
// ═══════════════════════════════════════════════════════════════
bool AriaHardwareInterface::open_serial_port()
{
  // TODO(Stage 4): Open serial port with termios configuration
  // serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY);
  // Configure baud rate, 8N1, no flow control
  RCLCPP_WARN(
    rclcpp::get_logger("AriaHardwareInterface"),
    "Serial port open: NOT IMPLEMENTED (Stage 4)");
  return false;
}

void AriaHardwareInterface::close_serial_port()
{
  if (serial_fd_ >= 0) {
    // close(serial_fd_);
    serial_fd_ = -1;
  }
}

bool AriaHardwareInterface::send_serial_command(const std::string & cmd)
{
  if (serial_fd_ < 0) {
    return false;
  }
  // TODO(Stage 4): write(serial_fd_, cmd.c_str(), cmd.size());
  (void)cmd;
  return true;
}

std::string AriaHardwareInterface::read_serial_response()
{
  // TODO(Stage 4): Read from serial_fd_ with timeout
  return "";
}

}  // namespace arm_control

// ── Register plugin with pluginlib ────────────────────────────
PLUGINLIB_EXPORT_CLASS(
  arm_control::AriaHardwareInterface,
  hardware_interface::SystemInterface)
