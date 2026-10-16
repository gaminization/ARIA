#!/bin/bash
# Restart the full ARIA sim stack headless. Usage: e2e_restart.sh [world_file]
WORLD=${1:-aria_tester_workspace.world}
pkill -9 gzserver; pkill -9 gzclient
pkill -9 -f "ros2 launch"
for p in vision_agent depth_agent tracking_agent affordance_agent attention_agent llm_planning_agent \
  reachability_agent skill_agent control_agent safety_agent memory_agent world_model_agent learning_agent \
  evaluation_agent dialogue_agent openvla_executor task_manager memory_manager health_monitor \
  detection_node depth_node ik_node visual_servo_node manual_control_node bag_recorder_node \
  industrial_workcell_node conveyor_part_spawner; do pkill -9 -f "$p"; done
sleep 2
rm -f /dev/shm/fastrtps* /dev/shm/sem.fastrtps* 2>/dev/null
cd /home/gaminizer/Projects/ARIA
source /opt/ros/humble/setup.bash; source install/setup.bash
nohup setsid ros2 launch arm_bringup aria_full_u3.launch.py world:=$WORLD gui:=false > /tmp/aria_launch.log 2>&1 &
echo started
