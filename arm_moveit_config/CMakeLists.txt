cmake_minimum_required(VERSION 3.8)
project(arm_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(PROGRAMS
  scripts/cable_scene_updater.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
