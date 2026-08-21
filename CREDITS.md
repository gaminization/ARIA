# CREDITS

ARIA uses assets and draws inspiration from the following open-source projects:

## Robot Arm Meshes

### [smart-methods/arduino_robot_arm](https://github.com/smart-methods/arduino_robot_arm)
- **Used**: STL meshes for the 5-DoF robotic arm (Base, Waist, Arm links, Gripper)
- **License**: Not specified
- **Description**: ROS packages for planning and executing motion trajectories for a robot arm in simulation and real-life. The robot arm uses MoveIt for kinematics via the KDL solver.

### [smart-methods/arduino_robot_arm (with gripper)](https://github.com/smart-methods/arduino_robot_arm)
- **Used**: Gripper STL meshes (Gripper1, Gripper2, gear1, gear2), URDF joint origins and transforms
- **License**: Not specified
- **Description**: Extended version of the arduino_robot_arm with a fully controllable gripper mechanism.

## Object Assets

### [personalrobotics/pr_assets](https://github.com/personalrobotics/pr_assets)
- **Used**: Graspable object meshes (bowl, glass, plate, fuze_bottle, plastic variants)
- **License**: BSD
- **Description**: Repository of OpenRAVE resources used by the Personal Robotics Lab, including meshes and KinBody specifications for objects commonly encountered by HERB and ADA robots.
- **Contributors**: Michael Koval, Aaron Walsman, Personal Robotics Lab at University of Washington

## Table & Furniture Models

### [code-iai/iai_table_robot_description](https://github.com/code-iai/iai_table_robot_description)
- **Used**: Reference for table-mounted robot workspace design
- **License**: Not specified
- **Description**: URDF description of table-mounted UR5 robots in the IAI lab.

## Simulation Scenarios

### [niteshjha08/warebots](https://github.com/niteshjha08/warebots)
- **Used**: Reference for warehouse simulation, multi-robot coordination patterns, DH parameter conventions
- **License**: Not specified
- **Description**: Warehouse Robotic Automation System — models and simulates multiple robots coordinating for pick-and-place, packaging, and labelling tasks.

### [smart-methods/robot_scenarios_sim](https://github.com/smart-methods/robot_scenarios_sim)
- **Used**: Gazebo model references (shelf, kitchen, table_set, tray models)
- **License**: Not specified
- **Description**: Collection of Gazebo simulation scenario models including kitchen, living room, storage, and shelf environments.

## Grasping & Manipulation Reference

### [gym-grasp](https://github.com/)
- **Used**: Reference for RL-based grasping task design (LiftObject, OpenDoor, OpenDrawer, PourCup)
- **License**: Not specified
- **Description**: Isaac Gym environments for robotic grasping and dexterous hand manipulation, providing reinforcement learning environments powered by Isaac Gym.

## Utility Tools

### [ros-industrial/DH2URDF](https://github.com/)
- **Used**: Reference for DH parameter to URDF conversion methodology
- **License**: MIT
- **Description**: JavaScript tool for converting Denavit-Hartenberg parameters to URDF format.

### [puppet](https://github.com/)
- **Used**: Reference for teleoperation GUI patterns
- **License**: BSD
- **Description**: ROS package for puppet-style robot control via GUI sliders.

---

*All assets are used in compliance with their respective licenses. Files are copied (not moved) from the reference repositories as requested.*
