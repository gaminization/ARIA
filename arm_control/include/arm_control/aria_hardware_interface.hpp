// ═══════════════════════════════════════════════════════════════
// ARIA Hardware Interface — Header
// Sim/Hardware switchable ros2_control SystemInterface
// In sim: passthrough (Gazebo handles physics via gz_ros2_control)
// In hardware: communicates with ESP32 via serial JSON protocol
// ═══════════════════════════════════════════════════════════════
#ifndef ARM_CONTROL__ARIA_HARDWARE_INTERFACE_HPP_
#define ARM_CONTROL__ARIA_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace arm_control
{

/// Number of actuated joints on the ARIA arm
constexpr size_t NUM_JOINTS = 6;

/// Joint names in kinematic chain order
const std::vector<std::string> JOINT_NAMES = {
  "waist_joint",
  "shoulder_joint",
  "elbow_joint",
  "wrist_pitch_joint",
  "wrist_roll_joint",
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
  bool use_sim_{true};                        // true = Gazebo, false = ESP32 hardware
  std::string serial_port_{"/dev/ttyUSB0"};   // Hardware serial port
  int baud_rate_{115200};                     // Hardware baud rate

  // ── Joint state storage ─────────────────────────────────
  std::vector<double> hw_positions_;          // Current positions [rad]
  std::vector<double> hw_velocities_;         // Current velocities [rad/s]
  std::vector<double> hw_commands_;           // Commanded positions [rad]

  // ── Hardware serial (for REAL mode, Stage 4) ────────────
  int serial_fd_{-1};                         // Serial file descriptor

  // ── Internal helpers ────────────────────────────────────
  bool open_serial_port();
  void close_serial_port();
  bool send_serial_command(const std::string & cmd);
  std::string read_serial_response();
};

}  // namespace arm_control

#endif  // ARM_CONTROL__ARIA_HARDWARE_INTERFACE_HPP_
