# CREDITS & ATTRIBUTION

ARIA uses assets, 3D meshes, world models, and draws architectural design patterns from the following open-source projects:

---

## 1. Robot Arm & Gripper CAD Models

### [smart-methods/arduino_robot_arm](https://github.com/smart-methods/arduino_robot_arm)
- **Files Copied**:
  - `arm_description/meshes/stl/Base.stl` (Base housing)
  - `arm_description/meshes/stl/Waist.stl` (Waist bracket)
  - `arm_description/meshes/stl/Arm 01.stl` (Upper arm link)
  - `arm_description/meshes/stl/Arm 02.stl` (Forearm link)
- **Role**: Base 3D geometry, joint hierarchy, and dimensional measurements for the 5-DoF robotic manipulator.
- **Original Authors**: Smart Methods Robotic Team.

### [smart-methods/arduino_robot_arm_gripper](https://github.com/smart-methods/arduino_robot_arm)
- **Files Copied**:
  - `arm_description/meshes/stl/Arm 03.stl` (Wrist pitch/roll mount)
  - `arm_description/meshes/stl/Gripper1.002.stl` (Primary drive finger pair)
  - `arm_description/meshes/stl/Gripper2.stl` (Opposing gear mimic finger pair)
  - `arm_description/meshes/stl/gear1.stl` & `gear2.stl` (Gear drive mechanism)
- **Role**: CAD joint origins, transforms, rotation axes, and mesh scale factors for precision kinematic alignment and gear-driven gripper control.

---

## 2. Laboratory Workstation & Optical Table

### [code-iai/iai_table_robot_description](https://github.com/code-iai/iai_table_robot_description)
- **Files Copied**:
  - `arm_description/meshes/table/complete_table.dae` (Industrial aluminum optical breadboard table)
  - `arm_description/meshes/table/complete_table.stl` (Collision model)
  - `arm_description/meshes/table/base_to_optical_table.stl` (CNC aluminum robot mounting adapter plate)
- **Role**: High-fidelity optical workstation environment and robotic arm table mounting fixture in Gazebo simulation.
- **Original Authors**: Institute for Artificial Intelligence (IAI), University of Bremen.

---

## 3. Realistic Scenario Props & Domestic Manipulation

### [smart-methods/robot_scenarios_sim](https://github.com/smart-methods/robot_scenarios_sim)
- **Files Copied**:
  - `arm_description/meshes/objects/plate.dae` (Ceramic dining plate)
  - `arm_description/meshes/objects/tray.dae` + texture maps (Sorting and serving tray)
  - `arm_description/meshes/objects/Dishes_Mug.dae` (Ceramic coffee mug)
  - `arm_description/meshes/objects/Green_Bottle.dae` + textures (Glass beverage bottle)
  - `arm_description/meshes/furniture/shelf.dae` (Workshop wall shelving unit)
- **Role**: Visual and physical collision assets for table-top object manipulation, sorting, and scene understanding.

---

## 4. Graspable Objects & Manipulation Assets

### [personalrobotics/pr_assets](https://github.com/personalrobotics/pr_assets)
- **Files Copied**:
  - `arm_description/meshes/objects/bowl.stl` (Ceramic cereal bowl)
  - `arm_description/meshes/objects/fork.stl` (Stainless steel fork)
  - `arm_description/meshes/objects/cube_mesh.stl` (Precision calibration and sorting cube)
  - `arm_description/meshes/objects/fuze_bottle_visual.dae` (Juice bottle)
- **Role**: OpenRAVE-compatible graspable meshes with realistic contact friction parameters.
- **License**: BSD-3-Clause
- **Original Authors**: Michael Koval, Aaron Walsman, Personal Robotics Lab, University of Washington.

---

## 5. Simulation Scenarios & Multi-Robot Architecture

### [niteshjha08/warebots](https://github.com/niteshjha08/warebots)
- **Role**: Architectural reference for multi-stage automation, Gazebo world physics tuning, and DH parameter conventions.
- **Original Authors**: Nitesh Jha.

---

## 6. Reinforcement Learning & Grasping Environments

### [gym-grasp](https://github.com/)
- **Role**: Reference for RL reward design, grasp reachability validation, and task formulation (Lift, Pour, Bin Sorting).

---

## 7. Kinematic & Teleoperation Tooling

### [ros-industrial/DH2URDF](https://github.com/ros-industrial/DH2URDF)
- **Role**: Reference tool for translating Denavit-Hartenberg parameter tables to standard URDF format.
- **License**: MIT

### [puppet](https://github.com/)
- **Role**: Reference for responsive slider-based teleoperation GUI design.
- **License**: BSD

---

*Note: All external assets are copied (not moved) from the reference repositories to preserve repository integrity. Attribution is documented in accordance with open-source licensing.*
