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

# Patch namespace: rosidl generates arm_planner in dist-packages (for msgs),
# but our source modules live in site-packages. Both __init__.py files need
# pkgutil.extend_path so Python merges both directories into one namespace.
set(_NS_PATCH "import pkgutil\\n__path__ = pkgutil.extend_path(__path__, __name__)\\n")
install(CODE "
  set(_DIST_INIT \"\${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/arm_planner/__init__.py\")
  set(_SITE_INIT \"\${CMAKE_INSTALL_PREFIX}/lib/python3.10/site-packages/arm_planner/__init__.py\")
  foreach(_INIT_FILE \${_DIST_INIT} \${_SITE_INIT})
    if(EXISTS \${_INIT_FILE})
      file(READ \${_INIT_FILE} _CONTENT)
      string(FIND \"\${_CONTENT}\" \"pkgutil\" _FOUND)
      if(_FOUND EQUAL -1)
        file(WRITE \${_INIT_FILE} \"import pkgutil\\n__path__ = pkgutil.extend_path(__path__, __name__)\\n\${_CONTENT}\")
      endif()
    endif()
  endforeach()
")

# ── Config ─────────────────────────────────────────────────
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  OPTIONAL
)

# ── Prompts ────────────────────────────────────────────────
install(DIRECTORY prompts/
  DESTINATION share/${PROJECT_NAME}/prompts
  OPTIONAL
)

# ── Data and logs directories ─────────────────────────────
install(DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/data
  OPTIONAL
)

# ── Scripts ────────────────────────────────────────────────
install(PROGRAMS
  scripts/upgrade_u1_install.sh
  DESTINATION lib/${PROJECT_NAME}
  OPTIONAL
)
install(DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
  OPTIONAL
)
)

ament_package()
