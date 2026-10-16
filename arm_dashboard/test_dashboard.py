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
from urllib.error import HTTPError, URLError

try:
    import websockets
    WEBSOCKETS_AVAILABLE = True
except ImportError:
    WEBSOCKETS_AVAILABLE = False


class ARIADashboardTest(unittest.TestCase):
    BASE_URL = "http://localhost:8000"
    WS_URL = "ws://localhost:8000"

    _server_proc = None

    @classmethod
    def setUpClass(cls):
        # Check if already running
        try:
            with urllib.request.urlopen(f"{cls.BASE_URL}/api/state", timeout=1.0) as resp:
                if resp.status == 200:
                    return
        except Exception:
            pass

        # Start dashboard server in background
        import sys, subprocess, os
        app_path = os.path.join(os.path.dirname(__file__), "app.py")
        env = os.environ.copy()
        env["ARIA_DASHBOARD_PORT"] = "8000"
        cls._server_proc = subprocess.Popen(
            [sys.executable, app_path],
            env=env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

        start = time.time()
        ready = False
        while time.time() - start < 10.0:
            try:
                with urllib.request.urlopen(f"{cls.BASE_URL}/api/state", timeout=1.0) as resp:
                    if resp.status == 200:
                        ready = True
                        break
            except Exception:
                time.sleep(0.5)

        if not ready:
            if cls._server_proc:
                cls._server_proc.terminate()
                cls._server_proc = None
            raise RuntimeError("Dashboard server failed to start within 10s")

    @classmethod
    def tearDownClass(cls):
        if cls._server_proc:
            try:
                cls._server_proc.terminate()
                cls._server_proc.wait(timeout=5)
            except Exception:
                pass
            cls._server_proc = None

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
            self.assertEqual(
                data["joints"]["names"],
                ["waist_joint", "shoulder_joint", "elbow_joint",
                 "wrist_pitch_joint", "gripper_joint"],
            )

            if not data["ros_connected"]:
                self.assertEqual(data["vision"]["detected_objects"], [])
                self.assertEqual(data["health"]["servo_health"], [])
                self.assertIsNone(data["health"]["fps_top"])
                self.assertIsNone(data["task"]["confidence"])

    def test_02_api_metrics(self):
        """Verify GET /api/metrics returns valid telemetry structure."""
        req = urllib.request.Request(f"{self.BASE_URL}/api/metrics")
        with urllib.request.urlopen(req, timeout=2.0) as response:
            self.assertEqual(response.status, 200)
            data = json.loads(response.read().decode())
            self.assertIn("recent_runs", data)
            self.assertIn("failure_breakdown", data)
            self.assertIn("overall_success_rate", data)
            self.assertIsInstance(data["recent_runs"], list)
            self.assertEqual(data["total_runs"], len(data["recent_runs"]) if data["total_runs"] <= 50 else data["total_runs"])
            if data["total_runs"] == 0:
                self.assertIsNone(data["overall_success_rate"])

    def assert_service_unavailable(self, endpoint, payload=None):
        req = urllib.request.Request(
            f"{self.BASE_URL}{endpoint}",
            data=json.dumps(payload or {}).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with self.assertRaises(HTTPError) as error:
            urllib.request.urlopen(req, timeout=3.0)
        self.assertEqual(error.exception.code, 503)

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
        """Do not report a command accepted when the task-manager service is offline."""
        self.assert_service_unavailable("/api/command", {"command": "Sort workcell objects"})

    def test_06_api_estop(self):
        """An unavailable E-stop must not be reported as successfully engaged."""
        self.assert_service_unavailable("/api/estop")

    def test_07_api_release_estop(self):
        """An unavailable release service must not be reported as successful."""
        self.assert_service_unavailable("/api/release_estop")

    def test_08_api_joint(self):
        """Manual commands require the actual ROS service."""
        self.assert_service_unavailable(
            "/api/joint",
            {"joint": "waist_joint", "angle_deg": 10.0, "speed": 30.0},
        )

    def test_09_api_approve_and_reject(self):
        """Approval endpoints report unavailable while disconnected."""
        for endpoint in ["/api/approve", "/api/reject"]:
            self.assert_service_unavailable(endpoint)

    def test_10_api_joint_array_requires_exact_actuator_count(self):
        """Reject malformed arrays instead of trimming phantom or excess axes."""
        req = urllib.request.Request(
            f"{self.BASE_URL}/api/joints",
            data=json.dumps({"angles_deg": [0.0] * 4}).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with self.assertRaises(HTTPError) as error:
            urllib.request.urlopen(req, timeout=2.0)
        self.assertEqual(error.exception.code, 422)

    def test_11_websocket_streaming(self):
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
