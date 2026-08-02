#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='arm_planner', executable='task_manager', name='task_manager', output='screen'),
        Node(package='arm_agents', executable='planning_agent', name='planning_agent', output='screen'),
        Node(package='arm_agents', executable='world_model_agent', name='world_model_agent', output='screen'),
        Node(package='arm_planner', executable='memory_manager', name='memory_manager', output='screen')
    ])
