#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Dialogue Agent — LifecycleNode
Natural language interface. User commands + approval flow +
status updates + failure explanations.
═══════════════════════════════════════════════════════════════
"""
import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from std_msgs.msg import String, Bool
from arm_planner.msg import TaskState
from arm_planner.state_bus import StateBus

class DialogueAgent(LifecycleNode):
    """
    Natural language interface for ARIA.

    Incoming: /aria/command (user text commands)
    Outgoing:
      /aria/dialogue/output (String: for dashboard)
      /aria/dialogue/requires_input (Bool)

    Features:
      - Status updates during execution
      - Failure explanations in plain language
      - Approval flow when confidence is low
    """

    def __init__(self):
        super().__init__('dialogue_agent')
        self.bus = StateBus(self)
        self.last_status = ''
        self.last_action_idx = -1

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("DialogueAgent: CONFIGURING")
        self.output_pub = self.create_publisher(String, '/aria/dialogue/output', 10)
        self.input_pub = self.create_publisher(Bool, '/aria/dialogue/requires_input', 10)
        self.bus.on_change('task', self._on_task)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("DialogueAgent: ACTIVATED")
        self._say("ARIA ready. Send commands to /aria/command.")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def _say(self, text: str):
        """Publish a dialogue output."""
        msg = String()
        msg.data = text
        self.output_pub.publish(msg)
        self.get_logger().info(f"[DIALOGUE] {text}")

    def _set_requires_input(self, needs: bool):
        msg = Bool()
        msg.data = needs
        self.input_pub.publish(msg)

    def _on_task(self, msg: TaskState):
        """React to task state changes and generate dialogue."""
        status = msg.task_status

        # Status transitions
        if status != self.last_status:
            self.last_status = status

            if status == 'PLANNING':
                self._say(f"🤔 Understanding command: \"{msg.current_command}\"...")

            elif status == 'EXECUTING':
                n = len(msg.action_queue)
                self._say(f"🚀 Executing plan: {n} actions queued. "
                          f"Goal: {msg.current_goal}")

            elif status == 'PAUSED':
                self._handle_approval_request(msg)

            elif status == 'RECOVERY':
                if msg.failure_log:
                    last_fail = msg.failure_log[-1]
                    self._say(
                        f"⚠ Problem encountered: {last_fail.failure_type}. "
                        f"Cause: {last_fail.cause}. Attempting recovery...")

            elif status == 'COMPLETE':
                self._say(f"✅ Task complete: \"{msg.current_command}\"")
                self._set_requires_input(False)

            elif status == 'FAILED':
                self._handle_failure(msg)

        # Progress updates during execution
        if status == 'EXECUTING':
            self._report_progress(msg)

    def _handle_approval_request(self, msg: TaskState):
        """Generate approval request dialogue."""
        if not msg.awaiting_user_approval:
            return

        reason = ""
        for action in msg.action_queue:
            if action.status == 'PENDING' and action.confidence < 0.75:
                reason = (
                    f"I'm about to execute '{action.action_type}' on "
                    f"'{action.target_object}', but my confidence is "
                    f"only {action.confidence:.0%}. "
                    f"Reason: {action.reasoning}")
                break

        if not reason:
            reason = f"Overall task confidence: {msg.confidence:.0%}"

        self._say(
            f"⏸ Paused — I need your approval.\n"
            f"{reason}\n"
            f"Should I proceed? "
            f"Call /aria/approve (yes) or /aria/reject (no)")
        self._set_requires_input(True)

    def _handle_failure(self, msg: TaskState):
        """Generate human-readable failure explanation."""
        if msg.failure_log:
            last_fail = msg.failure_log[-1]
            explanation = self._explain_failure(last_fail)
            self._say(f"❌ Task failed: \"{msg.current_command}\"\n{explanation}")
        else:
            self._say(f"❌ Task failed: \"{msg.current_command}\"")
        self._set_requires_input(False)

    def _explain_failure(self, failure) -> str:
        """Convert failure type to natural language."""
        explanations = {
            'MISSED_OBJECT': (
                f"I couldn't grasp the object. My gripper closed but "
                f"didn't make contact. This might be because the object "
                f"position was slightly off. Try adjusting lighting."),
            'OBJECT_SLIPPED': (
                f"I had the object but it slipped during transport. "
                f"The grip force might be too low for this object."),
            'IK_FAILURE': (
                f"I can't reach that position. It might be outside "
                f"my workspace or blocked by cables."),
            'COLLISION': (
                f"I couldn't find a safe path. The workspace might "
                f"be too cluttered. Try moving some objects."),
            'PERCEPTION_ERROR': (
                f"I couldn't see the object clearly enough. "
                f"Detection confidence was too low. "
                f"Try improving lighting or camera angle."),
            'TRACKING_LOST': (
                f"I lost track of the object. It may have been "
                f"moved or occluded."),
            'TIMEOUT': (
                f"The action took too long. Something may be stuck."),
        }
        return explanations.get(
            failure.failure_type,
            f"Failure type: {failure.failure_type}. Cause: {failure.cause}")

    def _report_progress(self, msg: TaskState):
        """Report action progress during execution."""
        for i, action in enumerate(msg.action_queue):
            if action.status == 'COMPLETE' and i > self.last_action_idx:
                self.last_action_idx = i
                self._say(f"  ✓ {action.action_type}({action.target_object})")
            elif action.status == 'EXECUTING' and i > self.last_action_idx:
                self._say(f"  → {action.action_type}({action.target_object})...")

def main(args=None):
    rclpy.init(args=args)
    node = DialogueAgent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
