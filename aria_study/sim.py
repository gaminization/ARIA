"""Headless Gazebo session management for the study."""
from __future__ import annotations

import os
import signal
import subprocess
import time
from pathlib import Path
from typing import Optional

from . import provenance

LOG_DIR = provenance.ROOT / "data" / "journal" / "raw"


class SimSession:
    def __init__(
        self,
        world: str = "aria_industrial_workcell.world",
        enable_attach: bool = False,
        enable_cameras: bool = True,
        gui: bool = False,
        log_name: str = "sim",
    ):
        self.world = world
        self.enable_attach = enable_attach
        self.enable_cameras = enable_cameras
        self.gui = gui
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        self.log_path = LOG_DIR / f"{log_name}.log"
        self.proc: Optional[subprocess.Popen] = None

    def start(self, timeout: float = 120.0) -> "SimSession":
        env = dict(os.environ)
        env["ARIA_ENABLE_ATTACH"] = "true" if self.enable_attach else "false"
        env["ARIA_ENABLE_CAMERAS"] = "true" if self.enable_cameras else "false"
        env["LIBGL_ALWAYS_SOFTWARE"] = env.get("LIBGL_ALWAYS_SOFTWARE", "1")
        env.setdefault("DISPLAY", ":0")
        cmd = [
            "ros2", "launch", "arm_bringup", "sim.launch.py",
            "gui:=true" if self.gui else "gui:=false",
            f"world:={self.world}",
        ]
        self._log_f = open(self.log_path, "w", encoding="utf-8")
        self.proc = subprocess.Popen(
            cmd,
            stdout=self._log_f,
            stderr=subprocess.STDOUT,
            env=env,
            start_new_session=True,
            text=True,
        )
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self.proc.poll() is not None:
                raise RuntimeError(
                    f"sim launch exited early ({self.proc.returncode}); see {self.log_path}"
                )
            try:
                text = self.log_path.read_text(errors="ignore")
            except FileNotFoundError:
                text = ""
            if "ARIA Simulation ready" in text:
                return self
            time.sleep(0.5)
        raise TimeoutError(f"sim not ready after {timeout}s; see {self.log_path}")

    def stop(self, timeout: float = 15.0) -> None:
        if not self.proc:
            return
        if self.proc.poll() is None:
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                self.proc.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
                self.proc.wait(timeout=5)
        try:
            self._log_f.close()
        except Exception:
            pass
        # gzserver sometimes survives a launch shutdown; make sure the next
        # experiment gets a clean world.
        subprocess.run(["pkill", "-9", "-f", "gzserver"], check=False,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(0.5)

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.stop()
        return False
