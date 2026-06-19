cmake_minimum_required(VERSION 3.8)
project(arm_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Dependencies ──────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# ── Hardware Interface Library ────────────────────────────
add_library(aria_hardware_interface SHARED
  src/aria_hardware_interface.cpp
)
target_include_directories(aria_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(aria_hardware_interface
  rclcpp
  hardware_interface
  pluginlib
  sensor_msgs
  trajectory_msgs
)

# Export hardware interface plugin
pluginlib_export_plugin_description_file(hardware_interface aria_hardware_plugin.xml)

# ── Install ───────────────────────────────────────────────
install(TARGETS aria_hardware_interface
  EXPORT aria_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install Python scripts as executables
install(PROGRAMS
  scripts/manual_control_node.py
  scripts/keyboard_control.py
  scripts/joint_slider_gui.py
  scripts/trajectory_generator.py
  scripts/grasp_executor.py
  scripts/visual_servo_node.py
  scripts/force_estimator_node.py
  scripts/kinematic_calibration_node.py
  scripts/servo_sync_node.py
  scripts/teach_mode_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Rename executables to remove .py extension
install(CODE "
  execute_process(COMMAND mv \${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/manual_control_node.py \${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/manual_control_node)
  execute_process(COMMAND mv \${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/servo_sync_node.py \${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/servo_sync_node)
  execute_process(COMMAND mv \${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/teach_mode_node.py \${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/teach_mode_node)
")

ament_export_include_directories(include)
ament_export_libraries(aria_hardware_interface)
ament_export_dependencies(
  rclcpp
  hardware_interface
  pluginlib
  sensor_msgs
  trajectory_msgs
)

ament_package()
