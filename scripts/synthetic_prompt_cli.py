#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════════════════
PROJECT ARIA: SYNTHETIC PROMPT CLI DISPATCHER
═══════════════════════════════════════════════════════════════════════════════
Allows operator to send natural language prompts to ARIA synthetic system
from the command line or via an interactive REPL shell.

Usage:
  # Single prompt execution:
  python3 scripts/synthetic_prompt_cli.py "wave"
  python3 scripts/synthetic_prompt_cli.py "stack the jenga tower"
  python3 scripts/synthetic_prompt_cli.py "pick up the mug by its handle"
  python3 scripts/synthetic_prompt_cli.py "put the banana in the pan"
  python3 scripts/synthetic_prompt_cli.py "sort the boxes"
  python3 scripts/synthetic_prompt_cli.py "switch conveyor speeds"

  # Interactive REPL session:
  python3 scripts/synthetic_prompt_cli.py
═══════════════════════════════════════════════════════════════════════════════
"""

import sys
import time
import argparse
import rclpy
from rclpy.node import Node
from arm_interfaces.srv import SendCommand
from std_msgs.msg import String


class SyntheticPromptClient(Node):
    def __init__(self):
        super().__init__("synthetic_prompt_client")
        self.cli_command = self.create_client(SendCommand, "/aria/command")

    def send_prompt(self, command_str: str, timeout_s: float = 15.0) -> bool:
        if not self.cli_command.wait_for_service(timeout_sec=min(5.0, timeout_s)):
            print("  ⚠️ Service /aria/command not found. Ensure synthetic system is launched.")
            return False

        req = SendCommand.Request()
        req.command = command_str.strip()
        future = self.cli_command.call_async(req)

        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < timeout_s:
            rclpy.spin_once(self, timeout_sec=0.1)
            if future.done():
                resp = future.result()
                if resp.accepted:
                    print(f"  ✓ Accepted: {resp.message} (Task ID: {resp.task_id})")
                    return True
                else:
                    print(f"  ✗ Rejected: {resp.message}")
                    return False

        print("  ⚠️ Service call timed out.")
        return False


def main():
    parser = argparse.ArgumentParser(description="ARIA Synthetic Command Prompt Dispatcher")
    parser.add_argument("prompt", nargs="*", help="Natural language prompt string to send (e.g. 'wave', 'stack mugs')")
    args = parser.parse_args()

    rclpy.init()
    client = SyntheticPromptClient()

    try:
        if args.prompt:
            prompt_str = " ".join(args.prompt)
            print(f"  Sending prompt to /aria/command: '{prompt_str}'...")
            client.send_prompt(prompt_str)
        else:
            print("╔═══════════════════════════════════════════════════════════════╗")
            print("║   🤖 ARIA SYNTHETIC INTERACTIVE PROMPT SHELL                  ║")
            print("║   Type any prompt (e.g. 'wave', 'stack jenga', 'sort boxes')   ║")
            print("║   Type 'help' for options, or 'exit' / Ctrl+C to quit.        ║")
            print("╚═══════════════════════════════════════════════════════════════╝")
            while rclpy.ok():
                try:
                    line = input("\n[ARIA Synthetic] >>> ").strip()
                    if not line:
                        continue
                    if line.lower() in ("exit", "quit", "q"):
                        break
                    if line.lower() in ("help", "?"):
                        print("""
Available Prompts:
  Tester World:
    - "wave"
    - "nod" / "say no"
    - "drink coffee"
    - "pick every object"
    - "pick up [banana|duck|plate|bowl|pan|glass|mug|...]"
    - "pick up the mug using its handle"
    - "stack the mugs"
    - "put the banana in the pan"
    - "put the orange ball in the glass"
    - "smaller in bigger" (duck in bowl)
    - "stack the jenga tower"
    - "sort by color"
    - "360 scan"
    - "pedestal transfer"
    - "wipe table"
    - "go home"
  Industrial World:
    - "sort boxes" (blue vs red defect)
    - "stack boxes" (pallet)
    - "drop boxes on the ground"
    - "switch conveyor speeds" (all speeds)
    - "press estop button"
    - "continuous infeed"
    - "go home"
                        """)
                        continue
                    client.send_prompt(line)
                except (EOFError, KeyboardInterrupt):
                    break
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
