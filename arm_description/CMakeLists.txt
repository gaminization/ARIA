cmake_minimum_required(VERSION 3.8)
project(arm_description)

find_package(ament_cmake REQUIRED)

# Install URDF/Xacro files and mesh assets
install(DIRECTORY
  urdf
  meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
