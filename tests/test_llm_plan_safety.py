from types import SimpleNamespace

from arm_planner.llm_planning_agent import LLMPlanningAgent


class _ChainOfThoughtSink:
    def add_chain_of_thought(self, _message):
        pass


def _enforce(plan, command):
    agent = SimpleNamespace(bus=_ChainOfThoughtSink())
    return LLMPlanningAgent._enforce_plan_invariants(agent, plan, command)


def test_button_actuation_plan_never_grasps_the_control():
    plan = {
        'actions': [
            {'skill': 'locate', 'parameters': {'object_id': 'emergency_stop_test_button'}},
            {'skill': 'reachability', 'parameters': {'object_id': 'emergency_stop_test_button'}},
            {'skill': 'plan_grasp', 'parameters': {'object_id': 'emergency_stop_test_button'}},
            {'skill': 'execute_grasp', 'parameters': {'object_id': 'emergency_stop_test_button'}},
        ],
    }

    constrained = _enforce(
        plan, 'Press the reachable emergency stop test button on the workcell')

    assert [
        action['skill'] for action in constrained['actions']
    ] == ['locate', 'reachability', 'press', 'verify']
    assert all(
        action['parameters']['object_id'] == 'emergency_stop_test_button'
        for action in constrained['actions']
    )


def test_non_actuation_grasp_plan_is_preserved():
    plan = {
        'actions': [
            {'skill': 'locate', 'parameters': {'object_id': 'red_cube'}},
            {'skill': 'plan_grasp', 'parameters': {'object_id': 'red_cube'}},
            {'skill': 'execute_grasp', 'parameters': {'object_id': 'red_cube'}},
        ],
    }

    constrained = _enforce(plan, 'Pick up the red cube')

    assert [
        action['skill'] for action in constrained['actions']
    ] == ['locate', 'plan_grasp', 'execute_grasp', 'lift', 'verify']
