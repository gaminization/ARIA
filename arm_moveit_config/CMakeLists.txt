cmake_minimum_required(VERSION 3.8)
project(arm_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# MoveIt Setup Assistant metadata: tells MoveItConfigsBuilder where the
# URDF (arm_description) and SRDF live.
install(FILES .setup_assistant
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/cable_scene_updater.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
