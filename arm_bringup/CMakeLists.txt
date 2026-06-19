cmake_minimum_required(VERSION 3.8)
project(arm_bringup)

find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install world files
install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install config files (RViz, etc.)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install scripts
install(PROGRAMS
  scripts/validate_stage1.py
  scripts/validate_stage2.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
