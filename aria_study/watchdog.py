"""A small 2.0 s watchdog for a blocking language-model call.

The ARIA stack had no watchdog: the only configured timeout was a 90 s HTTP
timeout.  This module implements the pre-registered mechanism as a separate,
testable component.  It does not cancel the call; it records that the call
exceeded the deadline so the caller can fall back.  The call itself runs in a
worker thread, and the watchdog polls at ``poll`` seconds.
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Optional


@dataclass
class WatchdogResult:
    result: Any
    error: Optional[BaseException]
    call_start: float
    call_end: float
    fire_time: Optional[float]
    fired: bool
    call_duration_s: float
    detection_latency_s: Optional[float]
    detection_excess_s: Optional[float]

    def as_dict(self) -> dict:
        return {
            "fired": bool(self.fired),
            "call_duration_s": float(self.call_duration_s),
            "detection_latency_s": (
                float(self.detection_latency_s)
                if self.detection_latency_s is not None
                else float("nan")
            ),
            "detection_excess_s": (
                float(self.detection_excess_s)
                if self.detection_excess_s is not None
                else float("nan")
            ),
        }


class LLMWatchdog:
    """Poll a blocking call and fire once it exceeds ``timeout`` seconds."""

    def __init__(self, timeout: float = 2.0, poll: float = 0.01):
        if timeout <= 0:
            raise ValueError("timeout must be positive")
        if poll <= 0:
            raise ValueError("poll must be positive")
        self.timeout = float(timeout)
        self.poll = float(poll)

    def run(self, fn: Callable[..., Any], *args, **kwargs) -> WatchdogResult:
        box: dict = {"result": None, "error": None}

        def target() -> None:
            try:
                box["result"] = fn(*args, **kwargs)
            except BaseException as exc:  # noqa: BLE001 - reported to caller
                box["error"] = exc

        start = time.monotonic()
        worker = threading.Thread(target=target, daemon=True)
        worker.start()
        fire_time: Optional[float] = None
        while worker.is_alive():
            now = time.monotonic()
            if fire_time is None and now - start >= self.timeout:
                fire_time = now
            time.sleep(self.poll)
        worker.join()
        end = time.monotonic()
        latency = (fire_time - start) if fire_time is not None else None
        excess = (latency - self.timeout) if latency is not None else None
        return WatchdogResult(
            result=box["result"],
            error=box["error"],
            call_start=start,
            call_end=end,
            fire_time=fire_time,
            fired=fire_time is not None,
            call_duration_s=end - start,
            detection_latency_s=latency,
            detection_excess_s=excess,
        )
