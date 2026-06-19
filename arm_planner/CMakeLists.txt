cmake_minimum_required(VERSION 3.8)
project(arm_planner)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# ── Message generation ────────────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObjectDetection.msg"
  "msg/WorldObject.msg"
  "msg/FailureEvent.msg"
  "msg/SemanticRelation.msg"
  "msg/Action.msg"
  "msg/ServoHealth.msg"
  "msg/VisionState.msg"
  "msg/MemoryState.msg"
  "msg/TaskState.msg"
  "msg/HealthState.msg"
  DEPENDENCIES geometry_msgs builtin_interfaces
)

# ── Python module ─────────────────────────────────────────
# ament_python_install_package(${PROJECT_NAME})
install(DIRECTORY arm_planner/
  DESTINATION lib/python3.10/site-packages/arm_planner
)

# ── Data and logs directories ─────────────────────────────
install(DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/data
  OPTIONAL
)

ament_package()
