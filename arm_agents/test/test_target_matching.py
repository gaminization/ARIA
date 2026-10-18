from pathlib import Path
from types import SimpleNamespace
import time
import unittest

import numpy as np
try:
    from ikpy.chain import Chain
    IKPY_AVAILABLE = True
except Exception:  # noqa: BLE001 — ikpy pulls in jax, which can fail
    Chain = None
    IKPY_AVAILABLE = False
from arm_planner.msg import WorldObject
from arm_agents.skill_agent import (
    SkillAgent,
    _attachment_matches_target,
    _cached_pose_for_target,
    _grasp_pitch_for_approach,
    _grasp_wrist_targets,
    _target_pose_cache_key,
)


class TargetMatchingTests(unittest.TestCase):
    def test_target_pose_cache_key_does_not_confuse_grasp_and_destination(self):
        blue_cylinder_pose = object()
        dish_pose = object()
        pose_cache = {
            _target_pose_cache_key("blue_cylinder"): blue_cylinder_pose,
            _target_pose_cache_key("sorting dish"): dish_pose,
        }

        self.assertIs(
            _cached_pose_for_target(pose_cache, "blue cylinder"),
            blue_cylinder_pose,
        )
        self.assertIs(
            _cached_pose_for_target(pose_cache, "sorting_dish"),
            dish_pose,
        )
        self.assertNotEqual(
            _target_pose_cache_key("blue_cylinder"),
            _target_pose_cache_key("sorting_dish"),
        )

    @unittest.skipUnless(IKPY_AVAILABLE, "ikpy/jax unavailable in this environment")
    def test_top_down_grasp_and_approach_poses_are_kinematically_reachable(self):
        urdf_path = (
            Path(__file__).resolve().parents[2]
            / "arm_ik"
            / "config"
            / "aria_arm.urdf"
        )
        chain = Chain.from_urdf_file(
            str(urdf_path),
            active_links_mask=[False, True, True, True, True, False],
        )
        targets = _grasp_wrist_targets(
            0.070,
            -0.140,
            0.014,
            _grasp_pitch_for_approach("top_down"),
            0.09,
        )

        for target in targets:
            pose = SimpleNamespace(
                header=SimpleNamespace(frame_id="base_link"),
                pose=SimpleNamespace(
                    position=SimpleNamespace(
                        x=target[0], y=target[1], z=target[2],
                    ),
                ),
            )
            joints = SkillAgent._solve_ik_kinematic(
                SimpleNamespace(_ik_chain=chain),
                pose,
            )
            self.assertIsNotNone(joints)
            fk = chain.forward_kinematics(
                np.asarray([0.0, *joints[:4], 0.0])
            )[:3, 3]
            self.assertLess(np.linalg.norm(fk - target), 0.005)

    def test_attachment_verification_accepts_the_requested_entity(self):
        self.assertTrue(_attachment_matches_target(
            "red_cube",
            "Securely attached workpiece: red_cube_1 (dist: 0.02 m)",
        ))

    def test_attachment_verification_rejects_an_unrelated_entity(self):
        self.assertFalse(_attachment_matches_target(
            "red_cube",
            "Securely attached workpiece: sorting_box (dist: 0.02 m)",
        ))

    def test_attachment_verification_supports_known_object_aliases(self):
        self.assertTrue(_attachment_matches_target(
            "blue cup",
            "Securely attached workpiece: blue_mug_2 (dist: 0.02 m)",
        ))

    def test_underscore_target_matches_class_and_color_tokens(self):
        target = SimpleNamespace(
            class_name="cube",
            name="cube_1",
            color="red",
            affordance_regions=[],
        )
        self.assertGreaterEqual(
            SkillAgent._match_target_object("red_cube", target),
            60.0,
        )

    def test_explicit_object_type_does_not_match_on_color_alone(self):
        red_mug = SimpleNamespace(
            class_name="mug",
            name="mug_1",
            color="red",
            affordance_regions=[],
        )
        self.assertEqual(
            SkillAgent._match_target_object("red_cube", red_mug),
            0.0,
        )

    def test_explicit_color_rejects_conflicting_candidate(self):
        blue_cube = SimpleNamespace(
            class_name="cube",
            name="cube_1",
            color="blue",
            affordance_regions=[],
        )
        self.assertEqual(
            SkillAgent._match_target_object("red_cube", blue_cube),
            0.0,
        )

    def test_plural_target_matches_singular_class(self):
        target = SimpleNamespace(
            class_name="cube",
            name="cube_1",
            color="red",
            affordance_regions=[],
        )
        self.assertGreaterEqual(
            SkillAgent._match_target_object("red cubes", target),
            35.0,
        )

    def test_semantic_alias_matches_cup_to_mug(self):
        mug = SimpleNamespace(
            class_name="mug",
            name="mug_1",
            color="blue",
            affordance_regions=[],
        )
        self.assertGreaterEqual(
            SkillAgent._match_target_object("blue cup", mug),
            35.0,
        )

    def test_geometric_reach_rejects_false_cube_pose(self):
        self.assertFalse(SkillAgent._within_geometric_reach(0.204, 0.360))
        self.assertTrue(SkillAgent._within_geometric_reach(0.16, 0.0))

    def test_stale_detection_stream_is_not_live_evidence(self):
        now = time.monotonic()
        self.assertTrue(SkillAgent._vision_stream_is_fresh(now - 0.5, now))
        self.assertFalse(SkillAgent._vision_stream_is_fresh(now - 2.0, now))
        self.assertFalse(SkillAgent._vision_stream_is_fresh(None, now))

    def test_stale_memory_pose_requires_a_nearby_live_detection(self):
        remembered = WorldObject()
        remembered.class_name = "cube"
        remembered.name = "cube_1"
        remembered.color = "red"
        remembered.last_known_pose.pose.position.x = 0.204
        remembered.last_known_pose.pose.position.y = 0.360

        live_red_cube = SimpleNamespace(
            class_name="cube",
            name="cube_960_280",
            color="red",
            affordance_regions=[],
            pose_3d=SimpleNamespace(
                pose=SimpleNamespace(
                    position=SimpleNamespace(x=0.16, y=0.0),
                ),
            ),
        )

        self.assertFalse(SkillAgent._memory_candidate_matches_live_detections(
            "red_cube", remembered, [live_red_cube],
        ))
        remembered.last_known_pose.pose.position.x = 0.16
        remembered.last_known_pose.pose.position.y = 0.0
        self.assertTrue(SkillAgent._memory_candidate_matches_live_detections(
            "red_cube", remembered, [live_red_cube],
        ))


if __name__ == "__main__":
    unittest.main()
