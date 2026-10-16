import numpy as np
import yaml

from arm_ik.ik_benchmark_node import generate_benchmark_poses
from arm_ik.ik_solvers.aria_analytical_ik import (
    JOINT_LIMITS,
    forward_kinematics,
)


def test_benchmark_poses_use_four_arm_axes_and_match_forward_kinematics():
    poses = generate_benchmark_poses(n_poses=8, seed=17)

    assert len(poses) == 8
    for pose in poses:
        joints = np.asarray(pose["ground_truth_joints"])
        assert joints.shape == (4,)
        assert np.all(joints >= JOINT_LIMITS[:, 0])
        assert np.all(joints <= JOINT_LIMITS[:, 1])

        expected_position = forward_kinematics(joints)[:3, 3]
        np.testing.assert_allclose(pose["position"], expected_position, atol=1e-12)


def test_checked_in_benchmark_poses_have_four_arm_axes():
    with open("arm_ik/data/benchmark_poses.yaml", encoding="utf-8") as pose_file:
        data = yaml.safe_load(pose_file)

    assert data["poses"]
    assert all(len(pose["ground_truth_joints"]) == 4 for pose in data["poses"])
