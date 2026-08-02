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
# Note: ament_python_install_package conflicts with rosidl_generate_interfaces,
# so we install manually to the same dist-packages path ament uses.
install(DIRECTORY arm_planner/
  DESTINATION local/lib/python3.10/dist-packages/arm_planner
)

# Install executable nodes
install(PROGRAMS 
  arm_planner/task_manager.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME task_manager
)
install(PROGRAMS 
  arm_planner/memory_manager.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME memory_manager
)
install(PROGRAMS 
  arm_planner/health_monitor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME health_monitor
)

# ── Data and logs directories ─────────────────────────────
install(DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/data
  OPTIONAL
)

# ── Scripts ───────────────────────────────────────────────
install(DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
  OPTIONAL
)

ament_package()
