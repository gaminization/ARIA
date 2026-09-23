#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Dashboard Integration & Regression Test Suite
Validates backend REST endpoints, WebSocket streaming, and state schema.
═══════════════════════════════════════════════════════════════
"""
import asyncio
import json
import time
import unittest
import urllib.request
import urllib.parse
from urllib.error import URLError

try:
    import websockets
    WEBSOCKETS_AVAILABLE = True
except ImportError:
    WEBSOCKETS_AVAILABLE = False


class ARIADashboardTest(unittest.TestCase):
    BASE_URL = "http://localhost:8000"
    WS_URL = "ws://localhost:8000"

    @classmethod
    def setUpClass(cls):
        # Wait up to 5s for the dashboard server to respond
        start = time.time()
        ready = False
        while time.time() - start < 5.0:
            try:
                with urllib.request.urlopen(f"{cls.BASE_URL}/api/state", timeout=1.0) as resp:
                    if resp.status == 200:
                        ready = True
                        break
            except Exception:
                time.sleep(0.5)

    def test_01_api_state_schema(self):
        """Verify that all 5 core state fields exist in /api/state payload."""
        req = urllib.request.Request(f"{self.BASE_URL}/api/state")
        with urllib.request.urlopen(req, timeout=2.0) as response:
            self.assertEqual(response.status, 200)
            data = json.loads(response.read().decode())

            self.assertIn("vision", data)
            self.assertIn("task", data)
            self.assertIn("joints", data)
            self.assertIn("health", data)
            self.assertIn("memory", data)

            # Check sub-fields
            self.assertIn("detected_objects", data["vision"])
            self.assertIn("current_goal", data["task"])
            self.assertIn("current_angles", data["joints"])
            self.assertIn("servo_health", data["health"])
            self.assertIn("known_objects", data["memory"])

    def test_02_api_metrics(self):
        """Verify GET /api/metrics returns valid telemetry structure."""
        req = urllib.request.Request(f"{self.BASE_URL}/api/metrics")
        with urllib.request.urlopen(req, timeout=2.0) as response:
            self.assertEqual(response.status, 200)
            data = json.loads(response.read().decode())
            self.assertIn("recent_runs", data)
            self.assertIn("failure_breakdown", data)
            self.assertIn("overall_success_rate", data)
            self.assertTrue(len(data["recent_runs"]) > 0)

    def test_03_api_objects(self):
        """Verify GET /api/objects returns world model objects list."""
        req = urllib.request.Request(f"{self.BASE_URL}/api/objects")
        with urllib.request.urlopen(req, timeout=2.0) as response:
            self.assertEqual(response.status, 200)
            data = json.loads(response.read().decode())
            self.assertIn("objects", data)
            self.assertIsInstance(data["objects"], list)

    def test_04_api_logs(self):
        """Verify GET /api/logs returns last chain-of-thought events."""
        req = urllib.request.Request(f"{self.BASE_URL}/api/logs")
        with urllib.request.urlopen(req, timeout=2.0) as response:
            self.assertEqual(response.status, 200)
            data = json.loads(response.read().decode())
            self.assertIn("logs", data)
            self.assertIsInstance(data["logs"], list)

    def test_05_api_command(self):
        """Verify POST /api/command accepts an operator instruction."""
        payload = json.dumps({"command": "Sort workcell objects"}).encode('utf-8')
        req = urllib.request.Request(
            f"{self.BASE_URL}/api/command",
            data=payload,
            headers={"Content-Type": "application/json"},
            method="POST"
        )
        with urllib.request.urlopen(req, timeout=3.0) as response:
            self.assertEqual(response.status, 200)
            data = json.loads(response.read().decode())
            self.assertTrue(data.get("success"))

    def test_06_api_estop(self):
        """Verify POST /api/estop engages safety halt."""
        req = urllib.request.Request(
            f"{self.BASE_URL}/api/estop",
            data=b"{}",
            headers={"Content-Type": "application/json"},
            method="POST"
        )
        with urllib.request.urlopen(req, timeout=2.0) as response:
            self.assertEqual(response.status, 200)
            data = json.loads(response.read().decode())
            self.assertTrue(data.get("success"))

    def test_07_api_release_estop(self):
        """Verify POST /api/release_estop clears safety halt."""
        req = urllib.request.Request(
            f"{self.BASE_URL}/api/release_estop",
            data=b"{}",
            headers={"Content-Type": "application/json"},
            method="POST"
        )
        with urllib.request.urlopen(req, timeout=2.0) as response:
            self.assertEqual(response.status, 200)
            data = json.loads(response.read().decode())
            self.assertTrue(data.get("success"))

    def test_08_api_joint(self):
        """Verify POST /api/joint accepts manual joint target."""
        payload = json.dumps({"joint": "waist_joint", "angle_deg": 10.0, "speed": 30.0}).encode('utf-8')
        req = urllib.request.Request(
            f"{self.BASE_URL}/api/joint",
            data=payload,
            headers={"Content-Type": "application/json"},
            method="POST"
        )
        with urllib.request.urlopen(req, timeout=2.0) as response:
            self.assertEqual(response.status, 200)
            data = json.loads(response.read().decode())
            self.assertTrue(data.get("success"))

    def test_09_api_approve_and_reject(self):
        """Verify POST /api/approve and /api/reject endpoints."""
        for endpoint in ["/api/approve", "/api/reject"]:
            req = urllib.request.Request(
                f"{self.BASE_URL}{endpoint}",
                data=b"{}",
                headers={"Content-Type": "application/json"},
                method="POST"
            )
            with urllib.request.urlopen(req, timeout=2.0) as response:
                self.assertEqual(response.status, 200)
                data = json.loads(response.read().decode())
                self.assertIn("success", data)
                self.assertIn("message", data)

    def test_10_websocket_streaming(self):
        """Verify WebSocket /ws/state establishes connection and streams state."""
        if not WEBSOCKETS_AVAILABLE:
            self.skipTest("websockets library not installed in test environment")

        async def check_ws():
            async with websockets.connect(f"{self.WS_URL}/ws/state") as ws:
                msg = await asyncio.wait_for(ws.recv(), timeout=2.0)
                data = json.loads(msg)
                return data

        loop = asyncio.new_event_loop()
        try:
            data = loop.run_until_complete(check_ws())
            self.assertIn("vision", data)
            self.assertIn("task", data)
        finally:
            loop.close()


if __name__ == "__main__":
    unittest.main()
