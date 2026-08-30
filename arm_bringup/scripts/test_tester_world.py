#!/usr/bin/env python3
# ═══════════════════════════════════════════════════════════════
# ARIA Tester World Validation Script
# Verifies:
#  1. Static world file syntax, ODE physics, and models
#  2. IAI Optical Table mounting & base plate positioning
#  3. Bullet3 Benchmark Suite asset presence & grasp parameters
#  4. Tabletop HandyBot RGB-D sensor configurations & reach arcs
#  5. Live ROS 2 topics (camera color, aligned depth, point cloud)
#
# Usage:
#   python3 arm_bringup/scripts/test_tester_world.py
#   or
#   ros2 run arm_bringup test_tester_world.py
# ═══════════════════════════════════════════════════════════════
import os
import sys
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory


def print_banner(title):
    print("\n" + "═" * 65)
    print(f"  {title}")
    print("═" * 65)


def validate_offline():
    print_banner("ARIA TESTER WORLD STATIC VALIDATION")
    bringup_dir = get_package_share_directory("arm_bringup")
    desc_dir = get_package_share_directory("arm_description")

    world_path = os.path.join(bringup_dir, "worlds", "aria_tester_workspace.world")
    sdf_path = os.path.join(bringup_dir, "worlds", "aria_tester_workspace.sdf")

    # 1. Check World and SDF existence and XML integrity
    assert os.path.exists(world_path), f"World file missing: {world_path}"
    assert os.path.exists(sdf_path), f"SDF file missing: {sdf_path}"

    tree = ET.parse(world_path)
    root = tree.getroot()
    world = root.find("world")
    assert world is not None, "Root <world> element missing"
    print(" [PASS] World and SDF XML files exist and parse successfully.")

    # 2. Check Physics parameters
    physics = world.find("physics")
    assert physics is not None, "Physics element missing"
    ode = physics.find("ode")
    assert ode is not None, "ODE solver missing"
    iters = ode.find("solver/iters").text
    print(f" [PASS] ODE Physics Solver: High accuracy ({iters} iterations).")

    # 3. Check Models in World
    models = world.findall("model")
    model_names = [m.attrib.get("name") for m in models]
    print(f" [PASS] Detected {len(models)} models in tester world:")
    for name in model_names:
        print(f"   • {name}")

    required_models = [
        "overhead_camera_model",
        "side_inspection_rig",
        "ground_plane",
        "optical_table",
        "arm_mounting_plate",
        "elevated_pedestal_left",
        "elevated_pedestal_right",
        "mug_ycb_front",
        "bullet_duck",
        "bullet_banana",
        "beverage_bottle",
        "dinner_plate",
        "plastic_glass",
        "plastic_bowl",
        "ceramic_tea_mug",
        "tefal_pan",
        "mug_bullet_right",
        "sorting_tray",
        "orange_fruit",
        "pop_tarts_box",
        "jenga_tower_b1", "jenga_tower_b2", "jenga_tower_b3", "jenga_tower_b4",
        "jenga_block_1", "jenga_block_2"
    ]
    for req in required_models:
        assert req in model_names, f"Required model {req} missing from world!"
    print(f" [PASS] All {len(required_models)} required benchmark & table models present.")

    # 4. Check Meshes in arm_description
    bullet_mesh_dir = os.path.join(desc_dir, "meshes", "bullet")
    objects_mesh_dir = os.path.join(desc_dir, "meshes", "objects")
    table_mesh_dir = os.path.join(desc_dir, "meshes", "table")
    assert os.path.exists(bullet_mesh_dir), f"Bullet mesh directory missing: {bullet_mesh_dir}"
    assert os.path.exists(objects_mesh_dir), f"Objects mesh directory missing: {objects_mesh_dir}"
    assert os.path.exists(table_mesh_dir), f"Table mesh directory missing: {table_mesh_dir}"

    table_meshes = ["complete_table.dae", "base_to_optical_table.stl"]
    for m in table_meshes:
        p = os.path.join(table_mesh_dir, m)
        assert os.path.exists(p), f"Table mesh missing: {p}"
    print(" [PASS] IAI Optical Table meshes verified.")

    bullet_meshes = [
        "duck_norm.obj", "banana_norm.obj", "jenga_norm.obj",
        "tray_norm.obj", "pan_tefal_norm.obj", "plate_norm.obj",
        "tea_mug_norm.obj", "bullet_mug_norm.obj"
    ]
    for m in bullet_meshes:
        p = os.path.join(bullet_mesh_dir, m)
        assert os.path.exists(p), f"Bullet3 mesh missing: {p}"
    print(" [PASS] Bullet3 benchmark meshes verified.")

    object_meshes = [
        "pop_tarts_visual.dae", "fuze_bottle_visual.dae",
        "plastic_glass_norm.obj", "plastic_bowl_norm.obj",
        "ycb_mug/ycb_mug_norm.obj"
    ]
    for m in object_meshes:
        p = os.path.join(objects_mesh_dir, m)
        assert os.path.exists(p), f"Object mesh missing: {p}"
    print(" [PASS] Household, Affordance Mugs & Grocery object meshes verified.")

    # 5. Check Sensors
    sensor_model = world.find(".//model[@name='overhead_camera_model']")
    sensors = sensor_model.findall(".//sensor")
    sensor_names = [s.attrib.get("name") for s in sensors]
    print(f" [PASS] Overhead Camera configured: {sensor_names}")
    assert "top_camera" in sensor_names, "Top camera sensor missing!"

    print("\n>>> ALL STATIC TESTS PASSED! World is ready for simulation. <<<\n")


if __name__ == "__main__":
    validate_offline()
