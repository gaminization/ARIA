#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Upgrade U3 — Infrastructure Validation Suite
8 tests covering Docker, CI, tracking, bags, and object DB.

Usage:
  python3 arm_bringup/scripts/validate_upgrade_u3.py
═══════════════════════════════════════════════════════════════
"""
import json
import os
import shutil
import sqlite3
import subprocess
import sys
import tempfile
import time
from typing import Callable, Tuple

import yaml
import numpy as np


# ═══════════════════════════════════════════════════════════════
# Test framework
# ═══════════════════════════════════════════════════════════════
class TestResult:
    def __init__(self, name, passed, message, duration_s):
        self.name = name
        self.passed = passed
        self.message = message
        self.duration_s = duration_s


def run_test(name, test_fn) -> TestResult:
    print(f"\n  ┌─ Test: {name}")
    t0 = time.time()
    try:
        passed, msg = test_fn()
        dt = time.time() - t0
        icon = "✅ PASS" if passed else "❌ FAIL"
        print(f"  └─ {icon} ({dt:.1f}s): {msg}")
        return TestResult(name, passed, msg, dt)
    except Exception as e:
        dt = time.time() - t0
        print(f"  └─ ❌ ERROR ({dt:.1f}s): {e}")
        return TestResult(name, False, str(e), dt)


# ═══════════════════════════════════════════════════════════════
# Test 1: Docker files exist
# ═══════════════════════════════════════════════════════════════
def test_docker_files() -> Tuple[bool, str]:
    """Verify all Docker infrastructure files exist."""
    aria_dir = os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__))))

    required = [
        'docker/Dockerfile.base',
        'docker/Dockerfile.sim',
        'docker/Dockerfile.headless',
        'docker/Dockerfile.dashboard',
        'docker/entrypoint.sh',
        'docker/Makefile',
        'docker/NVIDIA_SETUP.md',
        'docker-compose.yml',
        'docker-compose.ci.yml',
    ]

    missing = []
    for f in required:
        path = os.path.join(aria_dir, f)
        if not os.path.exists(path):
            missing.append(f)

    if missing:
        return False, f"Missing: {', '.join(missing)}"

    # Validate docker-compose.yml
    compose_path = os.path.join(aria_dir, 'docker-compose.yml')
    with open(compose_path, 'r') as f:
        compose = yaml.safe_load(f)

    services = list(compose.get('services', {}).keys())
    required_svc = ['ros2_core', 'dashboard']
    for svc in required_svc:
        if svc not in services:
            return False, f"Missing service: {svc}"

    return True, f"All {len(required)} Docker files present, {len(services)} services"


# ═══════════════════════════════════════════════════════════════
# Test 2: Docker available
# ═══════════════════════════════════════════════════════════════
def test_docker_available() -> Tuple[bool, str]:
    """Check if Docker is installed and accessible."""
    try:
        result = subprocess.run(
            ['docker', '--version'],
            capture_output=True, text=True, timeout=5)
        if result.returncode != 0:
            return False, "Docker command failed"
        version = result.stdout.strip()
    except FileNotFoundError:
        return False, "Docker not installed"
    except subprocess.TimeoutExpired:
        return False, "Docker command timed out"

    # Check compose v2
    try:
        result = subprocess.run(
            ['docker', 'compose', 'version'],
            capture_output=True, text=True, timeout=5)
        compose_ok = result.returncode == 0
    except Exception:
        compose_ok = False

    compose_msg = "compose v2 OK" if compose_ok else "compose v2 missing"
    return True, f"{version} ({compose_msg})"


# ═══════════════════════════════════════════════════════════════
# Test 3: CI/CD pipeline files
# ═══════════════════════════════════════════════════════════════
def test_ci_pipeline() -> Tuple[bool, str]:
    """Verify CI/CD pipeline configuration."""
    aria_dir = os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__))))

    required = [
        '.github/workflows/aria_ci.yml',
        '.github/workflows/docker_build.yml',
        '.github/scripts/compare_benchmarks.py',
    ]

    missing = []
    for f in required:
        if not os.path.exists(os.path.join(aria_dir, f)):
            missing.append(f)

    if missing:
        return False, f"Missing: {', '.join(missing)}"

    # Validate CI workflow
    ci_path = os.path.join(aria_dir, '.github/workflows/aria_ci.yml')
    with open(ci_path, 'r') as f:
        ci = yaml.safe_load(f)

    jobs = list(ci.get('jobs', {}).keys())
    if 'build' not in jobs:
        return False, "CI missing 'build' job"
    if 'unit_tests' not in jobs:
        return False, "CI missing 'unit_tests' job"

    triggers = ci.get('on', {})
    has_push = 'push' in triggers
    has_schedule = 'schedule' in triggers

    return True, f"{len(jobs)} CI jobs, push={has_push}, schedule={has_schedule}"


# ═══════════════════════════════════════════════════════════════
# Test 4: Experiment tracker
# ═══════════════════════════════════════════════════════════════
def test_experiment_tracker() -> Tuple[bool, str]:
    """Test experiment tracking (offline mode)."""
    sys.path.insert(0, os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        '..', 'arm_learning'))

    from arm_learning.experiment_tracker import (
        ExperimentTracker, IKBenchmarkResults, SkillResult,
        SessionMetrics, get_system_config,
    )

    # Test with 'none' backend (no MLflow needed)
    tracker = ExperimentTracker(backend='none')
    assert tracker.backend_name == 'none'

    # Log IK benchmark
    results = IKBenchmarkResults(
        solver_name='test_solver', n_test_poses=100,
        success_rate=0.95, mean_solve_time_ms=2.1,
        p95_solve_time_ms=5.3, position_error_mm=0.8)
    run_id = tracker.log_ik_benchmark(results)
    assert run_id is not None

    # Log skill performance
    skill = SkillResult(
        skill_name='pick_up', success=True,
        duration_s=3.5, object_class='cup')
    tracker.log_skill_performance('pick_up', skill)

    # Log session summary
    session = SessionMetrics(
        session_id='test_001',
        start_time='2026-07-01T10:00:00',
        end_time='2026-07-01T10:30:00',
        tasks_attempted=10, tasks_completed=8,
        overall_success_rate=0.8)
    tracker.log_session_summary(session)

    # System config
    config = get_system_config()
    assert 'hostname' in config
    assert 'python' in config

    return True, f"Tracker OK (backend={tracker.backend_name}, system_config={len(config)} keys)"


# ═══════════════════════════════════════════════════════════════
# Test 5: Bag indexer
# ═══════════════════════════════════════════════════════════════
def test_bag_indexer() -> Tuple[bool, str]:
    """Test bag indexing and search."""
    sys.path.insert(0, os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        '..', 'arm_learning'))

    from arm_learning.bag_indexer import BagIndexer

    # Use temp DB
    with tempfile.TemporaryDirectory() as tmpdir:
        db_path = os.path.join(tmpdir, 'test_bag_index.db')
        indexer = BagIndexer(db_path=db_path)

        # Create mock bags with metadata
        bag_dir = os.path.join(tmpdir, '2026-07-01')
        os.makedirs(bag_dir)

        for i, name in enumerate(['session_100000_standard',
                                   'session_110000_full',
                                   'failure_120000_pick']):
            bag_path = os.path.join(bag_dir, name)
            os.makedirs(bag_path)
            # Write metadata
            meta = {
                'start_time': f'2026-07-01T{10+i}:00:00',
                'end_time': f'2026-07-01T{10+i}:30:00',
                'duration_s': 1800,
                'mode': 'sim',
                'recording_mode': name.split('_')[-1],
                'tasks_attempted': 5 + i,
                'objects_in_scene': ['red_cube', 'blue_cup'],
                'failure_events': [{'type': 'MISSED'}] if 'failure' in name else [],
            }
            meta_path = bag_path + '_metadata.yaml'
            with open(meta_path, 'w') as f:
                yaml.dump(meta, f)

        # Index
        count = indexer.index_directory(tmpdir)
        assert count == 3, f"Expected 3, indexed {count}"

        # Search
        all_bags = indexer.search()
        assert len(all_bags) == 3

        failures = indexer.search(failure_only=True)
        assert len(failures) == 1, f"Expected 1 failure, got {len(failures)}"

        cup_bags = indexer.search(object_class='blue_cup')
        assert len(cup_bags) == 3

        # Stats
        stats = indexer.get_stats()
        assert stats['total_bags'] == 3

    return True, f"Indexer OK: indexed {count}, search works, stats OK"


# ═══════════════════════════════════════════════════════════════
# Test 6: Bag to dataset conversion
# ═══════════════════════════════════════════════════════════════
def test_bag_to_dataset() -> Tuple[bool, str]:
    """Test bag-to-HDF5 dataset conversion."""
    sys.path.insert(0, os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        '..', 'arm_learning'))

    from arm_learning.bag_to_dataset import (
        convert_bag_to_hdf5, write_lerobot_hdf5, Episode,
    )

    with tempfile.TemporaryDirectory() as tmpdir:
        # Create test episodes
        episodes = []
        for i in range(3):
            ep = Episode(
                episode_id=i,
                images_top=[np.zeros((48, 64, 3), dtype=np.uint8)] * 10,
                images_wrist=[np.zeros((48, 64, 3), dtype=np.uint8)] * 10,
                joint_states=[np.random.randn(6).astype(np.float32) for _ in range(10)],
                joint_commands=[np.random.randn(6).astype(np.float32) for _ in range(10)],
                timestamps=list(np.linspace(0, 1, 10)),
                success=i < 2,  # 2 success, 1 failure
                task_label='pick_up',
                object_class='test_cube',
            )
            episodes.append(ep)

        # Write HDF5
        output = os.path.join(tmpdir, 'test_dataset.hdf5')
        write_lerobot_hdf5(episodes, output, fps=30)

        assert os.path.exists(output), "HDF5 not created"

        # Verify HDF5 structure
        import h5py
        with h5py.File(output, 'r') as f:
            assert 'metadata' in f
            assert f['metadata'].attrs['n_episodes'] == 3
            assert f['metadata'].attrs['robot_type'] == 'aria_6dof'

            assert 'episodes' in f
            ep0 = f['episodes/episode_000']
            assert 'observation.state' in ep0
            assert 'action' in ep0
            assert 'reward' in ep0
            assert ep0['observation.state'].shape == (10, 6)
            assert ep0.attrs['success'] == True

        size_kb = os.path.getsize(output) / 1024

    return True, f"Dataset OK: 3 episodes, HDF5 valid ({size_kb:.0f} KB)"


# ═══════════════════════════════════════════════════════════════
# Test 7: Object model database
# ═══════════════════════════════════════════════════════════════
def test_object_model_db() -> Tuple[bool, str]:
    """Test object model database operations."""
    sys.path.insert(0, os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        '..', 'arm_planner'))

    from arm_planner.object_model_db import ObjectModelDB, GraspParams

    aria_dir = os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__))))

    with tempfile.TemporaryDirectory() as tmpdir:
        db_path = os.path.join(tmpdir, 'test_objects.db')
        db = ObjectModelDB(db_path=db_path)

        # Import default library
        lib_path = os.path.join(
            aria_dir, 'arm_planner', 'data', 'object_library_default.yaml')
        count = db.import_library(lib_path)
        assert count >= 5, f"Expected ≥5 imports, got {count}"

        # Get model
        bottle = db.get_model('red_bottle')
        assert bottle is not None
        assert bottle.material == 'rigid_plastic'
        assert bottle.mass_kg > 0

        # Get grasp params
        params = db.get_grasp_params('blue_cup')
        assert isinstance(params, GraspParams)
        assert params.grip_force_scale < 1.0  # Ceramic = gentle

        # Register new object
        new_id = db.register_new_object('green_mug', {
            'mass_kg': 0.3,
            'material': 'ceramic',
            'fragile': True,
        })
        assert new_id > 0

        mug = db.get_model('green_mug')
        assert mug is not None
        assert mug.material == 'ceramic'
        assert mug.fragile == True

        # Update success rate
        for i in range(10):
            db.update_success_rate('red_bottle', success=(i < 8))

        bottle2 = db.get_model('red_bottle')
        assert bottle2.times_grasped == 10
        assert bottle2.grasp_success_rate > 0

        # Export
        export_path = os.path.join(tmpdir, 'export.yaml')
        n_exported = db.export_library(export_path)
        assert n_exported > 0
        assert os.path.exists(export_path)

        # Stats
        stats = db.get_stats()
        assert len(stats) > 0

        all_objects = db.list_objects()

    return True, (
        f"Object DB OK: imported {count}, registered green_mug, "
        f"tracked 10 grasps, exported {n_exported}, "
        f"total {len(all_objects)} objects")


# ═══════════════════════════════════════════════════════════════
# Test 8: Benchmark comparison
# ═══════════════════════════════════════════════════════════════
def test_benchmark_comparison() -> Tuple[bool, str]:
    """Test benchmark comparison tool."""
    aria_dir = os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__))))

    sys.path.insert(0, os.path.join(aria_dir, '.github', 'scripts'))

    # Import directly
    spec_path = os.path.join(
        aria_dir, '.github', 'scripts', 'compare_benchmarks.py')

    import importlib.util
    spec = importlib.util.spec_from_file_location(
        'compare_benchmarks', spec_path)
    cb = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(cb)

    # Test comparison logic
    baseline = {
        'ik_success_rate': 0.95,
        'ik_mean_solve_time_ms': 2.0,
        'ik_position_error_mm': 0.5,
    }

    # No regression
    current_good = {
        'ik_success_rate': 0.96,
        'ik_mean_solve_time_ms': 1.8,
        'ik_position_error_mm': 0.4,
    }
    all_pass, results = cb.compare_metrics(current_good, baseline, 0.10)
    assert all_pass, "Good metrics should pass"

    # Regression
    current_bad = {
        'ik_success_rate': 0.80,  # Dropped from 0.95
        'ik_mean_solve_time_ms': 2.0,
        'ik_position_error_mm': 0.5,
    }
    all_pass_bad, results_bad = cb.compare_metrics(current_bad, baseline, 0.10)
    assert not all_pass_bad, "Bad metrics should fail"

    return True, "Benchmark comparison: regression detection working"


# ═══════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════
def main():
    print("\n" + "═" * 54)
    print("  ARIA Upgrade U3 — Infrastructure Validation")
    print("═" * 54)

    tests = [
        ("Docker Files", test_docker_files),
        ("Docker Available", test_docker_available),
        ("CI/CD Pipeline", test_ci_pipeline),
        ("Experiment Tracker", test_experiment_tracker),
        ("Bag Indexer", test_bag_indexer),
        ("Bag to Dataset", test_bag_to_dataset),
        ("Object Model DB", test_object_model_db),
        ("Benchmark Comparison", test_benchmark_comparison),
    ]

    results = []
    for name, fn in tests:
        results.append(run_test(name, fn))

    n_pass = sum(1 for r in results if r.passed)
    n_total = len(results)

    print("\n" + "═" * 54)
    print(f"  Results: {n_pass}/{n_total} tests passed")
    print("═" * 54)
    for r in results:
        icon = "✅" if r.passed else "❌"
        print(f"  {icon} {r.name}: {r.message}")
    print("═" * 54)

    if n_pass == n_total:
        print("  🎉 U3 UPGRADE VALIDATION COMPLETE")
    else:
        print(f"  ⚠ {n_total - n_pass} test(s) failed")
    print("═" * 54 + "\n")

    return 0 if n_pass == n_total else 1


if __name__ == "__main__":
    sys.exit(main())
