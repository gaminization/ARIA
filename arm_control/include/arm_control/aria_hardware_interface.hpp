// ═══════════════════════════════════════════════════════════════
// ARIA Hardware Interface — Header
// Sim/Hardware switchable ros2_control SystemInterface
//
// In SIM mode the Gazebo physics plugin (gazebo_ros2_control)
// drives the joints and this class is unused.
//
// In HARDWARE mode this class bridges ros2_control to the real
// ESP32 arm. The ESP32 runs the micro-ROS firmware in
// `esp32_firmware/` (node `aria_esp32`), so the bridge talks to the
// exact same topics that firmware exposes:
//   * publishes /aria/servo_commands (JointState, degrees)
//   * subscribes /aria/servo_states  (JointState, degrees)
//   * publishes /aria/teach_mode     (Bool: true = servos limp)
// The micro_ros_agent (launched by hardware.launch.py) carries those
// topics over the ESP32's USB-serial or WiFi link.
// ═══════════════════════════════════════════════════════════════
#ifndef ARM_CONTROL__ARIA_HARDWARE_INTERFACE_HPP_
#define ARM_CONTROL__ARIA_HARDWARE_INTERFACE_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"

namespace arm_control
{

/// Number of actuated joints on the ARIA arm
constexpr size_t NUM_JOINTS = 5;

/// Four arm axes followed by the gripper
const std::vector<std::string> JOINT_NAMES = {
  "waist_joint",
  "shoulder_joint",
  "elbow_joint",
  "wrist_pitch_joint",
  "gripper_joint"
};

class AriaHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(AriaHardwareInterface)

  // ── Lifecycle callbacks ─────────────────────────────────
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  // ── Interface export ────────────────────────────────────
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // ── Read/Write cycle ────────────────────────────────────
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // ── Parameters ──────────────────────────────────────────
  bool use_sim_{true};                             // true = Gazebo, false = ESP32
  std::string command_topic_{"/aria/servo_commands"};  // To ESP32 (degrees)
  std::string state_topic_{"/aria/servo_states"};      // From ESP32 (degrees)
  std::string teach_topic_{"/aria/teach_mode"};        // To ESP32 (Bool)
  double state_timeout_s_{1.0};                    // Reject stale feedback

  // ── Joint state storage (radians / rad/s) ───────────────
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;
  std::vector<std::string> hw_joint_names_;

  // ── micro-ROS bridge (REAL mode) ────────────────────────
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr servo_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr teach_mode_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr servo_state_sub_;

  std::mutex state_mutex_;
  std::vector<double> latest_positions_;   // radians
  std::vector<double> latest_velocities_;  // rad/s (NaN = unknown)
  bool state_received_{false};
  std::chrono::steady_clock::time_point last_state_time_;

  /// Index of `name` in hw_joint_names_, or -1 when absent.
  int find_joint(const std::string & name) const;

  /// Called for every /aria/servo_states message.
  void state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
};

}  // namespace arm_control

#endif  // ARM_CONTROL__ARIA_HARDWARE_INTERFACE_HPP_
