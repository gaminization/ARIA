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

# Install nodes as executables
install(PROGRAMS
  arm_planner/task_manager.py
  arm_planner/memory_manager.py
  arm_planner/health_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

# Rename executables to remove .py extension
install(CODE "
  execute_process(COMMAND mv \${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/task_manager.py \${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/task_manager)
  execute_process(COMMAND mv \${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/memory_manager.py \${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/memory_manager)
  execute_process(COMMAND mv \${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/health_monitor.py \${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/health_monitor)
")

# ── Data and logs directories ─────────────────────────────
install(DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/data
  OPTIONAL
)

ament_package()
