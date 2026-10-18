from types import SimpleNamespace
import unittest

import numpy as np

from arm_agents.depth_agent import DepthAgent
from arm_agents.skill_agent import SkillAgent
from arm_planner.task_manager import DEFAULT_AUTO_APPROVE_AFTER_S


class _Bus:
    def __init__(self):
        self.messages = []

    def add_chain_of_thought(self, message):
        self.messages.append(message)


class _Publisher:
    def __init__(self):
        self.values = []

    def publish(self, message):
        self.values.append(message.data)


class TruthfulSkillTests(unittest.TestCase):
    def test_low_confidence_approval_does_not_timeout_to_automatic_approval(self):
        self.assertEqual(DEFAULT_AUTO_APPROVE_AFTER_S, 3.0)

    def test_motion_verification_requires_recorded_commanded_joints(self):
        agent = SimpleNamespace(
            bus=_Bus(),
            _last_commanded_joints=None,
            current_joints_rad=[0.0, 0.0, 0.0, 0.0, 0.0],
        )
        self.assertFalse(SkillAgent._motion_verification_ok(
            agent, SimpleNamespace(action_type="verify", target_object="",
                                    reasoning="")))

    def test_motion_verification_confirms_arm_reached_commanded_posture(self):
        agent = SimpleNamespace(
            bus=_Bus(),
            _last_commanded_joints=np.array([0.1, -0.2, 0.3, 0.0]),
            current_joints_rad=[0.1, -0.2, 0.3, 0.0, 0.0],
        )
        self.assertTrue(SkillAgent._motion_verification_ok(
            agent, SimpleNamespace(action_type="verify", target_object="",
                                    reasoning="Confirm the wave gesture completed")))

    def test_motion_verification_fails_when_arm_did_not_arrive(self):
        agent = SimpleNamespace(
            bus=_Bus(),
            _last_commanded_joints=np.array([0.5, 0.0, 0.0, 0.0]),
            current_joints_rad=[0.0, 0.0, 0.0, 0.0, 0.0],
        )
        self.assertFalse(SkillAgent._motion_verification_ok(
            agent, SimpleNamespace(action_type="verify", target_object="",
                                    reasoning="wave")))

    def test_unimplemented_manipulation_skills_fail_instead_of_claiming_success(self):
        agent = SimpleNamespace(bus=_Bus())
        action = SimpleNamespace(target_object="object")
        for skill in (
            SkillAgent._skill_push,
            SkillAgent._skill_pull,
            SkillAgent._skill_slide,
            SkillAgent._skill_roll,
            SkillAgent._skill_sweep,
        ):
            with self.subTest(skill=skill.__name__):
                self.assertFalse(skill(agent, action))

    def test_lift_refuses_unplanned_joint_fallback_when_ik_fails(self):
        bus = _Bus()
        agent = SimpleNamespace(
            bus=bus,
            _held_workpiece="red_cube",
            _is_holding_object=True,
            _cached_grasp_pose=SimpleNamespace(
                pose=SimpleNamespace(position=SimpleNamespace(
                    x=0.1, y=0.0, z=0.05,
                )),
            ),
            _solve_ik=lambda _pose: None,
            _move_joints=lambda *_args, **_kwargs: self.fail(
                "should not move without an IK solution"),
        )
        self.assertFalse(SkillAgent._skill_lift(
            agent, SimpleNamespace(target_object="red_cube")))

    def test_unwired_perception_skills_do_not_claim_verified_results(self):
        agent = SimpleNamespace(
            bus=_Bus(),
            _cached_target_pose=None,
            _skill_locate=lambda _action: False,
        )
        action = SimpleNamespace(
            target_object="clear glass",
            target_pose=None,
        )
        self.assertFalse(SkillAgent._skill_sam(agent, action))
        self.assertFalse(SkillAgent._skill_depth(agent, action))
        self.assertFalse(SkillAgent._skill_cleargrasp(agent, action))
        self.assertFalse(SkillAgent._skill_material(agent, action))

    def test_depth_agent_uses_measured_timings_not_fixed_constants(self):
        agent = SimpleNamespace(
            da_latency_ms=0.0,
            midas_latency_ms=0.0,
            latency_pub=_Publisher(),
        )
        DepthAgent._metrics_cb(agent, SimpleNamespace(data=[12.5, 0.0]))
        DepthAgent._metrics_cb(agent, SimpleNamespace(data=[0.0, 21.0]))
        DepthAgent._evaluate_models(agent)
        self.assertEqual(agent.da_latency_ms, 12.5)
        self.assertEqual(agent.midas_latency_ms, 21.0)
        self.assertEqual(agent.latency_pub.values, [12.5])

    def test_depth_agent_ignores_malformed_timing_messages(self):
        class _Logger:
            def __init__(self):
                self.warnings = []

            def warning(self, message):
                self.warnings.append(message)

        logger = _Logger()
        agent = SimpleNamespace(
            da_latency_ms=0.0,
            midas_latency_ms=0.0,
            get_logger=lambda: logger,
        )
        DepthAgent._metrics_cb(agent, SimpleNamespace(data=[10.0]))
        self.assertEqual(agent.da_latency_ms, 0.0)
        self.assertEqual(agent.midas_latency_ms, 0.0)
        self.assertEqual(len(logger.warnings), 1)


if __name__ == "__main__":
    unittest.main()
