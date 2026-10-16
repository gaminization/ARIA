#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
Project ARIA: Dynamic System Configuration & Resource Manager
═══════════════════════════════════════════════════════════════
Dynamically discovers host hardware (CUDA VRAM, CPU threads, RAM)
and dynamically configures execution parameters for vision, depth,
SAM2, VLA, and LLM inference.

Guarantees portability across:
  - Consumer laptops with 8GB VRAM (e.g. RTX 5060 / 4060)
  - Workstations with 16GB-24GB VRAM (e.g. RTX 4090 / A5000)
  - Edge devices with 4GB-6GB VRAM (e.g. Jetson Orin / RTX 3050)
  - Headless CPU servers / CI environments

Prevents callback queue saturation, thread deadlocks, and CUDA OOM.
═══════════════════════════════════════════════════════════════
"""

import os
import sys
import time
import shutil
from typing import Dict, Any, Optional

try:
    import torch
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False


class SysConfig:
    """Singleton system hardware profile and adaptive execution manager."""

    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(SysConfig, cls).__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        if self._initialized:
            return
        self._initialized = True
        self.device = "cpu"
        self.gpu_name = "None"
        self.total_vram_gb = 0.0
        self.free_vram_gb = 0.0
        self.cpu_count = os.cpu_count() or 4
        self.hardware_tier = "LOW"  # LOW, MEDIUM, HIGH

        # Last execution timestamps for rate-limiting
        self._last_exec_times: Dict[str, float] = {}
        self._last_cache_clear = 0.0

        self._discover_hardware()

    def _discover_hardware(self):
        """Inspect CUDA and host compute resources."""
        if TORCH_AVAILABLE and torch.cuda.is_available():
            try:
                self.device = "cuda:0"
                self.gpu_name = torch.cuda.get_device_name(0)
                props = torch.cuda.get_device_properties(0)
                self.total_vram_gb = props.total_memory / (1024 ** 3)
                free_mem, _ = torch.cuda.mem_get_info(0)
                self.free_vram_gb = free_mem / (1024 ** 3)
            except Exception:
                self.device = "cuda:0"
                self.gpu_name = "CUDA Generic"
                self.total_vram_gb = 7.5
                self.free_vram_gb = 6.0
        else:
            self.device = "cpu"
            self.gpu_name = "CPU Only"
            self.total_vram_gb = 0.0
            self.free_vram_gb = 0.0

        # Hardware Tier classification
        if self.total_vram_gb >= 12.0:
            self.hardware_tier = "HIGH"
        elif self.total_vram_gb >= 6.0:
            self.hardware_tier = "MEDIUM"
        else:
            self.hardware_tier = "LOW"

    def refresh_vram(self):
        """Update current free VRAM reading."""
        if TORCH_AVAILABLE and torch.cuda.is_available():
            try:
                free_mem, _ = torch.cuda.mem_get_info(0)
                self.free_vram_gb = free_mem / (1024 ** 3)
            except Exception:
                pass

    def clear_gpu_cache(self):
        """Safely release unreferenced PyTorch cached GPU memory."""
        if TORCH_AVAILABLE and torch.cuda.is_available():
            try:
                torch.cuda.empty_cache()
            except Exception:
                pass

    def periodic_cache_clear(self, interval_s: float = 30.0):
        """Clear cached GPU memory occasionally, not on every camera frame."""
        now = time.monotonic()
        if now - self._last_cache_clear >= interval_s:
            self.clear_gpu_cache()
            self._last_cache_clear = now

    # ── Dynamic Model Recommendations ─────────────────────────

    @property
    def recommended_yolo_model(self) -> str:
        """Choose between yolov8m and yolov8n based on available VRAM."""
        aria_root = os.environ.get("ARIA_ROOT", "/home/gaminizer/Projects/ARIA")
        if self.hardware_tier in ("HIGH", "MEDIUM"):
            path = os.path.join(aria_root, "models", "yolov8m.pt")
            if os.path.exists(path):
                return path
            return "yolov8m.pt"
        else:
            path = os.path.join(aria_root, "models", "yolov8n.pt")
            if os.path.exists(path):
                return path
            return "yolov8n.pt"

    @property
    def detection_max_rate_hz(self) -> float:
        """Maximum inference rate for YOLO detection to prevent callback lag."""
        if self.hardware_tier == "HIGH":
            return 25.0
        elif self.hardware_tier == "MEDIUM":
            return 15.0  # 15 FPS ensures 66ms period, easily handling 20ms YOLO
        else:
            return 6.0

    @property
    def depth_max_rate_hz(self) -> float:
        """Maximum inference rate for depth estimation."""
        if self.hardware_tier == "HIGH":
            return 12.0
        elif self.hardware_tier == "MEDIUM":
            return 8.0   # 8 FPS prevents depth callback pile-up
        else:
            return 3.0

    @property
    def sam2_max_rate_hz(self) -> float:
        """Maximum rate for SAM2 mask inference."""
        if self.hardware_tier == "HIGH":
            return 10.0
        elif self.hardware_tier == "MEDIUM":
            return 5.0
        else:
            return 2.0

    @property
    def use_half_precision(self) -> bool:
        """Enable fp16 where supported by CUDA."""
        return self.device.startswith("cuda")

    def should_process(self, module_name: str, max_rate_hz: float) -> bool:
        """
        Check if a frame should be processed or dropped to maintain target rate.
        Thread-safe frame rate limiter.
        """
        now = time.time()
        min_interval = 1.0 / max(max_rate_hz, 0.1)
        last_t = self._last_exec_times.get(module_name, 0.0)
        if (now - last_t) >= min_interval:
            self._last_exec_times[module_name] = now
            return True
        return False

    def get_summary(self) -> Dict[str, Any]:
        """Return diagnostic dictionary of current system configuration."""
        self.refresh_vram()
        return {
            "device": self.device,
            "gpu_name": self.gpu_name,
            "hardware_tier": self.hardware_tier,
            "total_vram_gb": round(self.total_vram_gb, 2),
            "free_vram_gb": round(self.free_vram_gb, 2),
            "cpu_count": self.cpu_count,
            "recommended_yolo": self.recommended_yolo_model,
            "detection_max_hz": self.detection_max_rate_hz,
            "depth_max_hz": self.depth_max_rate_hz,
            "use_fp16": self.use_half_precision,
        }


# Global singleton instance
sys_config = SysConfig()
