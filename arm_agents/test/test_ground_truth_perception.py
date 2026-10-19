"""Unit tests for simulator ground-truth perception fallback in SkillAgent.

These tests verify that when the learned perception stack (YOLO/depth) is
unavailable, the skill agent can still locate and register workspace objects
from the authoritative Gazebo ``/gazebo/model_states`` stream — without
faking success (poses are reachability-gated and ambiguity-checked).
"""

import math
import unittest
from types import SimpleNamespace

from geometry_msgs.msg import PoseStamped

from arm_agents.skill_agent import (
    SkillAgent,
    GAZEBO_WORKPIECE_CLASSES,
    GAZEBO_STATIC_MODELS,
)


def make_pose(x, y, z):
    pose = PoseStamped()
    pose.header.frame_id = 'world'
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = float(z)
    return pose


def make_agent(poses):
    """Build a minimal SkillAgent stand-in exposing the helpers under test."""
    agent = SimpleNamespace()
    agent._gazebo_model_poses = poses
    agent._within_geometric_reach = SkillAgent._within_geometric_reach
    agent._match_target_object = SkillAgent._match_target_object
    agent._gazebo_model_to_workpiece = SkillAgent._gazebo_model_to_workpiece
    agent._ground_truth_workpiece_entries = (
        SkillAgent._ground_truth_workpiece_entries.__get__(agent, type(agent)))
    agent._ground_truth_locate = (
        SkillAgent._ground_truth_locate.__get__(agent, type(agent)))
    # Attributes touched by _skill_locate's success path.
    agent._target_pose_cache = {}
    agent._cached_target_pose = None
    agent._cached_grasp_target_key = None
    agent._latest_vision_dets = []
    agent._last_detection_time = None
    agent._vision_stream_is_fresh = SkillAgent._vision_stream_is_fresh
    return agent


class GroundTruthModelMappingTests(unittest.TestCase):
    def test_known_workpieces_map_to_class_and_color(self):
        for model in ('mug_ycb_front', 'bullet_banana', 'orange_fruit',
                      'jenga_block_1', 'tefal_pan'):
            self.assertIn(model, GAZEBO_WORKPIECE_CLASSES)

    def test_static_and_environment_models_are_rejected(self):
        for model in ('ground_plane', 'optical_table', 'conveyor_system',
                      'aria_arm', 'emergency_stop_test_button'):
            self.assertIn(model, GAZEBO_STATIC_MODELS)
            self.assertIsNone(SkillAgent._gazebo_model_to_workpiece(model))

    def test_unmapped_environment_model_is_not_a_workpiece(self):
        self.assertIsNone(SkillAgent._gazebo_model_to_workpiece('mystery_fixture'))

    def test_unknown_model_with_known_token_is_accepted(self):
        self.assertEqual(
            SkillAgent._gazebo_model_to_workpiece('pop_tarts_box_2'),
            ('box', ''))

    def test_all_tester_world_objects_are_recognised(self):
        """Every object in the synthetic tester catalog must map to a workpiece."""
        from synthetic_tester_suite import SyntheticTesterSuite
        for model in SyntheticTesterSuite.ALL_OBJECTS:
            with self.subTest(model=model):
                self.assertIsNotNone(SkillAgent._gazebo_model_to_workpiece(model))


class GroundTruthEntryTests(unittest.TestCase):
    def test_entries_are_filtered_by_reachability(self):
        poses = {
            'orange_fruit': make_pose(0.18, -0.16, 0.6081),   # reachable
            'bullet_banana': make_pose(2.00, 0.00, 0.6081),   # out of reach
            'ground_plane': make_pose(0.0, 0.0, 0.0),
        }
        entries = SkillAgent._ground_truth_workpiece_entries(make_agent(poses))
        names = {e.name for e in entries}
        self.assertIn('orange_fruit', names)
        self.assertNotIn('bullet_banana', names)
        self.assertNotIn('ground_plane', names)

    def test_entries_expose_matching_fields(self):
        poses = {'mug_ycb_front': make_pose(0.22, -0.06, 0.6081)}
        entries = SkillAgent._ground_truth_workpiece_entries(make_agent(poses))
        self.assertEqual(len(entries), 1)
        entry = entries[0]
        self.assertEqual(entry.class_name, 'mug')
        self.assertEqual(entry.color, 'yellow')
        self.assertEqual(entry.last_known_pose.pose.position.x, 0.22)
        self.assertEqual(entry.affordance_regions, ['color:yellow'])


class GroundTruthLocateTests(unittest.TestCase):
    def setUp(self):
        self.poses = {
            'orange_fruit': make_pose(0.18, -0.16, 0.6081),
            'bullet_banana': make_pose(0.20, -0.12, 0.6081),
            'mug_ycb_front': make_pose(0.22, -0.06, 0.6081),
            'mug_bullet_right': make_pose(0.00, -0.22, 0.6081),
            'plastic_bowl': make_pose(-0.08, 0.24, 0.6081),
            'ground_plane': make_pose(0.0, 0.0, 0.0),
        }
        self.agent = make_agent(self.poses)

    def test_semantic_alias_resolves_to_model(self):
        name, pose, score = SkillAgent._ground_truth_locate(self.agent, 'orange ball')
        self.assertEqual(name, 'orange_fruit')
        self.assertAlmostEqual(pose.pose.position.x, 0.18)
        self.assertGreaterEqual(score, 35.0)

    def test_banana_resolves(self):
        name, _, _ = SkillAgent._ground_truth_locate(self.agent, 'pick up the banana')
        self.assertEqual(name, 'bullet_banana')

    def test_exact_model_name_resolves(self):
        name, _, _ = SkillAgent._ground_truth_locate(self.agent, 'mug_ycb_front')
        self.assertEqual(name, 'mug_ycb_front')

    def test_color_disqualifies_conflicting_candidates(self):
        # Requesting the blue mug must not resolve the yellow one.
        name, _, _ = SkillAgent._ground_truth_locate(self.agent, 'blue mug')
        self.assertEqual(name, 'mug_bullet_right')

    def test_missing_target_returns_none(self):
        self.assertIsNone(SkillAgent._ground_truth_locate(self.agent, 'a horse'))

    def test_ambiguous_request_is_not_guessed(self):
        # Two identical-color mugs with no disambiguating token.
        poses = {
            'mug_a': make_pose(0.20, -0.06, 0.6081),
            'mug_b': make_pose(0.20, 0.06, 0.6081),
        }
        agent = make_agent(poses)
        self.assertIsNone(SkillAgent._ground_truth_locate(agent, 'mug'))

    def test_out_of_reach_target_is_not_reported(self):
        poses = {'orange_fruit': make_pose(1.5, 0.0, 0.6081)}
        agent = make_agent(poses)
        self.assertIsNone(SkillAgent._ground_truth_locate(agent, 'orange'))


class GroundTruthScanTests(unittest.TestCase):
    def test_scan_registers_ground_truth_and_skips_physical_sweep(self):
        moves = []
        registered = []

        agent = SimpleNamespace(
            bus=SimpleNamespace(add_chain_of_thought=lambda *_a, **_k: None),
            _ground_truth_workpiece_entries=lambda: [
                SimpleNamespace(
                    name='bullet_banana', class_name='banana', color='yellow',
                    last_known_pose=make_pose(0.20, -0.12, 0.6081)),
                SimpleNamespace(
                    name='orange_fruit', class_name='orange', color='orange',
                    last_known_pose=make_pose(0.18, -0.16, 0.6081)),
            ],
            _register_discovered_object=lambda **kw: registered.append(kw),
            _update_object_lifecycle=lambda *a, **k: None,
            _match_target_object=SkillAgent._match_target_object,
            _move_joints=lambda *a, **k: moves.append(a) or True,
            _wait_for_arrival=lambda *a, **k: True,
            _latest_vision_dets=[],
            _cached_target_pose=None,
            _target_pose_cache={},
        )
        action = SimpleNamespace(target_object='', target_pose=None)
        self.assertTrue(SkillAgent._skill_scan_workspace(agent, action))
        self.assertEqual(len(registered), 2)
        self.assertEqual(moves, [], "ground truth must not require a physical sweep")

    def test_scan_resolves_requested_target_from_ground_truth(self):
        registered = []
        agent = SimpleNamespace(
            bus=SimpleNamespace(add_chain_of_thought=lambda *_a, **_k: None),
            _ground_truth_workpiece_entries=lambda: [
                SimpleNamespace(
                    name='bullet_banana', class_name='banana', color='yellow',
                    last_known_pose=make_pose(0.20, -0.12, 0.6081)),
                SimpleNamespace(
                    name='orange_fruit', class_name='orange', color='orange',
                    last_known_pose=make_pose(0.18, -0.16, 0.6081)),
            ],
            _register_discovered_object=lambda **kw: registered.append(kw),
            _update_object_lifecycle=lambda *a, **k: None,
            _match_target_object=SkillAgent._match_target_object,
            _move_joints=lambda *a, **k: True,
            _wait_for_arrival=lambda *a, **k: True,
            _latest_vision_dets=[],
            _cached_target_pose=None,
            _target_pose_cache={},
        )
        action = SimpleNamespace(target_object='banana', target_pose=None)
        self.assertTrue(SkillAgent._skill_scan_workspace(agent, action))
        self.assertAlmostEqual(action.target_pose.pose.position.x, 0.20, places=3)


class GroundTruthLocateSkillTests(unittest.TestCase):
    def test_skill_locate_resolves_from_ground_truth(self):
        bus = SimpleNamespace(
            add_chain_of_thought=lambda *_a, **_k: None,
            state=SimpleNamespace(vision=None, memory=None),
            publish_memory=lambda *_a, **_k: None,
        )
        agent = make_agent({
            'orange_fruit': make_pose(0.18, -0.16, 0.6081),
        })
        agent.bus = bus
        action = SimpleNamespace(
            action_type='locate', target_object='orange ball',
            target_pose=None, destination='')
        # The method mutates action.target_pose; ground truth must resolve it
        # without any camera polling.
        self.assertTrue(SkillAgent._skill_locate(agent, action))
        self.assertAlmostEqual(action.target_pose.pose.position.x, 0.18, places=3)


if __name__ == "__main__":
    unittest.main()
