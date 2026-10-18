#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA VLA Interface — Vision-Language-Action Model Integration
Unified interface, multiple backend implementations.
Backends: LeRobot ACT, OpenVLA, Pi0 (placeholder), Gr00t (placeholder)
═══════════════════════════════════════════════════════════════
"""
import os
import sys
import time
import types
import abc
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass, field

import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image, JointState
    from std_msgs.msg import Float64MultiArray
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


# ═══════════════════════════════════════════════════════════════
# LeRobot import shim
# ═══════════════════════════════════════════════════════════════
#
# lerobot 0.4.4 eagerly imports `lerobot.policies.groot` from
# `lerobot/policies/__init__.py`. That submodule crashes at import
# time with:
#     TypeError: non-default argument 'backbone_cfg' follows
#                default argument
# Because the failure is inside the package __init__, it makes every
# real policy (ACT, Pi0, ...) unimportable in that release.
#
# ARIA does not use the Gr00t policy. Rather than let the broken
# eager import run at all, we register `lerobot.policies` as a
# lightweight namespace package (pointing at the real directory) so
# the specific policy module can be imported directly. No policy
# behaviour is altered; only the unused eager `__init__` is bypassed.

_LEROBOT_POLICY_MODULES = {
    "act": "lerobot.policies.act.modeling_act",
    "pi0": "lerobot.policies.pi0.modeling_pi0",
    "groot": "lerobot.policies.groot.modeling_groot",
}
_LEROBOT_POLICY_CLASSES = {
    "act": "ACTPolicy",
    "pi0": "PI0Policy",
    "groot": "GrootPolicy",
}


def _ensure_lerobot_policies_package():
    """
    Make `lerobot.policies.<x>` importable without running its broken
    eager `__init__`. If the package is already imported we leave it be.
    """
    if "lerobot.policies" in sys.modules:
        return
    import lerobot

    policies_dir = os.path.join(os.path.dirname(lerobot.__file__), "policies")
    pkg = types.ModuleType("lerobot.policies")
    pkg.__path__ = [policies_dir]
    pkg.__package__ = "lerobot.policies"
    sys.modules["lerobot.policies"] = pkg


def load_lerobot_policy_class(policy: str):
    """
    Import a real lerobot policy class, tolerating the groot import bug.

    Raises ImportError if lerobot is not installed, or the underlying
    error if the policy genuinely cannot be imported.
    """
    import importlib

    _ensure_lerobot_policies_package()
    module_name = _LEROBOT_POLICY_MODULES[policy]
    class_name = _LEROBOT_POLICY_CLASSES[policy]
    module = importlib.import_module(module_name)
    return getattr(module, class_name)


# ═══════════════════════════════════════════════════════════════
# Data Structures
# ═══════════════════════════════════════════════════════════════

@dataclass
class VLAInput:
    """Input to any VLA backend."""
    top_camera_image: Optional[np.ndarray] = None       # H×W×3 uint8
    wrist_camera_image: Optional[np.ndarray] = None     # H×W×3 uint8
    joint_positions: Optional[np.ndarray] = None        # 4 arm joints (rad)
    joint_velocities: Optional[np.ndarray] = None       # 4 arm joints (rad/s)
    gripper_position: float = 0.0                        # 0=open, 1=closed
    instruction: str = ""                                # NL task instruction
    timestep: int = 0


@dataclass
class VLAOutput:
    """Output from any VLA backend."""
    joint_commands: np.ndarray = field(default_factory=lambda: np.zeros(4))
    gripper_command: float = 0.0       # 0=open, 1=closed
    confidence: float = 0.0
    is_delta: bool = True              # True if commands are deltas
    action_label: str = ""             # Predicted action label
    reasoning: str = ""                # Explanation if available
    # End-to-end policies (OpenVLA, ACT) do not expose a calibrated
    # success probability. When this is False, `confidence` must be read
    # only as an action-availability flag (1.0 = a valid action was
    # produced, 0.0 = the backend could not act), never as a probability.
    confidence_is_calibrated: bool = False


# ═══════════════════════════════════════════════════════════════
# Abstract Backend
# ═══════════════════════════════════════════════════════════════

class VLABackend(abc.ABC):
    """Abstract base class for VLA backends."""

    def __init__(self, name: str, model_path: str = ""):
        self.name = name
        self.model_path = model_path
        self.loaded = False
        self.inference_times: List[float] = []

    @abc.abstractmethod
    def load(self) -> bool:
        """Load model weights. Returns True on success."""
        ...

    @abc.abstractmethod
    def predict(self, vla_input: VLAInput) -> VLAOutput:
        """Run inference. Returns VLAOutput."""
        ...

    @abc.abstractmethod
    def unload(self):
        """Free GPU/memory resources."""
        ...

    @property
    def avg_inference_ms(self) -> float:
        if not self.inference_times:
            return 0.0
        return np.mean(self.inference_times[-100:]) * 1000

    def _time_inference(self, fn, *args, **kwargs):
        t0 = time.time()
        result = fn(*args, **kwargs)
        dt = time.time() - t0
        self.inference_times.append(dt)
        return result


# ═══════════════════════════════════════════════════════════════
# Backend 1: LeRobot ACT Policy
# ═══════════════════════════════════════════════════════════════

class _LerobotPolicyBackend(VLABackend):
    """
    Base class for VLA backends backed by a real lerobot policy.

    A checkpoint is mandatory. Instantiating a randomly-initialised
    policy and then reporting its behaviour would be meaningless, so the
    backend refuses to run without real weights.

    `confidence` is reported as an action-availability flag (1.0 = a
    valid action was produced), never as a calibrated success
    probability, because these policies do not expose one.
    """

    policy_name = ""
    policy_label = ""

    def __init__(self, model_path: str = ""):
        super().__init__(self.policy_label, model_path)
        self.policy = None

    def load(self) -> bool:
        try:
            import torch  # noqa: F401
            policy_cls = load_lerobot_policy_class(self.policy_name)
        except ImportError as e:
            print(f"[VLA] {self.policy_label} unavailable: {e}. "
                  "Install with: pip install lerobot")
            self.loaded = False
            return False
        except Exception as e:
            print(f"[VLA] {self.policy_label} import failed: {e}")
            self.loaded = False
            return False

        checkpoint = self.model_path
        if not checkpoint or not os.path.exists(checkpoint):
            print(
                f"[VLA] No {self.policy_label} checkpoint supplied "
                f"({checkpoint or '<none>'}). Refusing to run a "
                "randomly-initialised policy in a benchmark.")
            self.loaded = False
            return False

        try:
            self.policy = policy_cls.from_pretrained(checkpoint)
            self.policy.eval()
            self.loaded = True
            return True
        except Exception as e:
            print(f"[VLA] {self.policy_label} load failed: {e}")
            self.policy = None
            self.loaded = False
            return False

    def predict(self, vla_input: VLAInput) -> VLAOutput:
        if not self.loaded or self.policy is None:
            return VLAOutput(confidence=0.0, reasoning="Model not loaded")

        def _infer():
            import torch

            # Prepare observation batch (lerobot 0.4.x batch-dict API)
            batch = {}
            if vla_input.top_camera_image is not None:
                img = vla_input.top_camera_image.astype(np.float32) / 255.0
                img = np.transpose(img, (2, 0, 1))  # HWC → CHW
                batch["observation.images.top"] = torch.from_numpy(img).unsqueeze(0)

            if vla_input.wrist_camera_image is not None:
                img = vla_input.wrist_camera_image.astype(np.float32) / 255.0
                img = np.transpose(img, (2, 0, 1))
                batch["observation.images.wrist"] = torch.from_numpy(img).unsqueeze(0)

            arm_positions = (
                np.asarray(vla_input.joint_positions, dtype=np.float32)
                if vla_input.joint_positions is not None
                else np.zeros(4, dtype=np.float32)
            )
            if arm_positions.shape != (4,):
                raise ValueError(
                    "VLA input must contain exactly four arm joint positions")
            state = np.concatenate([
                arm_positions,
                [vla_input.gripper_position],
            ])
            batch["observation.state"] = torch.from_numpy(
                state.astype(np.float32)).unsqueeze(0)

            with torch.no_grad():
                action = self.policy.select_action(batch)

            return np.atleast_1d(
                action.squeeze().detach().cpu().numpy()).astype(np.float32)

        action = np.atleast_1d(np.asarray(
            self._time_inference(_infer), dtype=np.float32))

        output = VLAOutput()
        output.joint_commands = action[:4]
        output.gripper_command = (
            float(np.clip(action[4], 0.0, 1.0)) if action.size > 4 else 0.0)
        output.is_delta = True
        # The policy exposes no calibrated success probability. Report
        # action availability only; never a fabricated confidence number.
        output.confidence = 1.0 if action.size else 0.0
        output.confidence_is_calibrated = False
        output.reasoning = (
            f"{self.policy_label}: policy action, "
            f"inference={self.avg_inference_ms:.0f}ms "
            f"(no calibrated confidence exposed by the policy)")
        return output

    def unload(self):
        self.policy = None
        self.loaded = False
        import gc
        gc.collect()


class LeRobotBackend(_LerobotPolicyBackend):
    """
    LeRobot (Hugging Face) ACT policy backend.

    Loads a policy trained on ARIA's own recorded data.
    Input:  camera images + joint states
    Output: joint velocity commands (chunked actions)
    """

    policy_name = "act"
    policy_label = "LeRobot-ACT"

    def __init__(self, model_path: str = ""):
        super().__init__(model_path)
        self.action_chunk_size = 10
        self.action_buffer: List[np.ndarray] = []


# ═══════════════════════════════════════════════════════════════
# Backend 2: OpenVLA
# ═══════════════════════════════════════════════════════════════

# OpenVLA-7B's modeling code is pinned to a specific stack and calls
# APIs that later transformers releases removed (e.g.
# `tie_weights(recompute_mapping=...)`). The rest of ARIA needs the
# newer stack, so the two cannot share an environment.
_OPENVLA_TRANSFORMERS_SPEC = "4.40.1"
_OPENVLA_TIMM_VERSIONS = {"0.9.10", "0.9.11", "0.9.12", "0.9.16"}

# OpenVLA-7B ships its unnormalisation statistics inside the checkpoint
# config under `norm_stats`. `bridge_orig` is a real key present in that
# config; its action layout is:
#     [dx, dy, dz, droll, dpitch, dyaw, gripper]
# i.e. a 6-DoF end-effector delta plus a gripper command. ARIA is a
# 5-DoF joint-space arm, so a Cartesian delta is projected into joint
# space with an explicit differential-IK step (see
# `eef_delta_to_joint_delta`). No learned projection is claimed and none
# is fabricated.
_OPENVLA_DEFAULT_UNNORM_KEY = "bridge_orig"


def _load_aria_ik():
    """Import ARIA's authoritative FK/IK module by path; None if unavailable."""
    import importlib.util
    root = os.environ.get("ARIA_ROOT", os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__)))))
    path = os.path.join(root, "arm_ik", "arm_ik", "ik_solvers",
                        "aria_analytical_ik.py")
    if not os.path.exists(path):
        return None
    try:
        spec = importlib.util.spec_from_file_location("aria_analytical_ik", path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod
    except Exception:
        return None


def eef_delta_to_joint_delta(current_joints, eef_delta_xyz,
                             damping: float = 1e-3):
    """
    Project a Cartesian EEF translation delta into ARIA joint space with
    damped least squares on ARIA's own FK Jacobian.

    This is an explicit, uncalibrated differential-IK step, not a
    trained policy projection, and callers must treat it as such.
    Returns (joint_delta[4] float32, ok: bool).
    """
    mod = _load_aria_ik()
    if mod is None:
        return np.zeros(4, dtype=np.float32), False
    q = np.asarray(current_joints, dtype=float)
    if q.size < 4:
        return np.zeros(4, dtype=np.float32), False
    q = q[:4]
    J = mod._numerical_jacobian(q)  # 3x4
    dp = np.asarray(eef_delta_xyz, dtype=float).reshape(3)
    A = J.T @ J + damping * np.eye(4)
    dq = np.linalg.solve(A, J.T @ dp)
    for k in range(4):
        lo, hi = mod.JOINT_LIMITS[k]
        dq[k] = float(np.clip(q[k] + dq[k], lo, hi)) - q[k]
    return dq.astype(np.float32), True


class OpenVLABackend(VLABackend):
    """
    OpenVLA 7B backend.

    Design:
    - Uses the WRIST (gripper eye-in-hand) camera as primary visual input.
    - Uses the checkpoint's own unnormalisation statistics (`unnorm_key`),
      not hand-written ones.
    - Tries 8-bit quantisation (bitsandbytes) first, then bfloat16.
    - OpenVLA-7B requires transformers==4.40.1 and timm in
      {0.9.10, 0.9.11, 0.9.12, 0.9.16}. When the active environment cannot
      satisfy that, `load()` reports OpenVLA as UNAVAILABLE instead of
      degrading to random weights.
    - OpenVLA emits a 6-DoF Cartesian EEF delta plus gripper (bridge
      layout). For ARIA this is projected into joint space with an
      explicit differential-IK step; it is NOT a fine-tuned ARIA policy,
      and the backend labels it as uncalibrated.
    - Local model path takes priority over the HF hub download.
    """

    LOCAL_MODEL_PATH = os.path.join(
        # Walk up: vla_interface.py → arm_vla/ → install/.../arm_vla/ → ...
        # Use env var ARIA_ROOT if set, else fall back to known absolute path
        os.environ.get("ARIA_ROOT", "/home/gaminizer/Projects/ARIA"),
        "models", "openvla-7b"
    )

    def __init__(self, model_path: str = "openvla/openvla-7b",
                 unnorm_key: str = _OPENVLA_DEFAULT_UNNORM_KEY):
        super().__init__("OpenVLA", model_path)
        self.model = None
        self.processor = None
        self.unnorm_key = unnorm_key
        self._output_raw_dim = 7       # OpenVLA bridge layout

    def _resolve_model_path(self) -> str:
        """Prefer local download, fall back to HF hub ID."""
        local = self.LOCAL_MODEL_PATH
        if os.path.isdir(local) and os.path.exists(
                os.path.join(local, "config.json")):
            print(f"[OpenVLA] Using local model at {local}")
            return local
        print(f"[OpenVLA] Local model not found at {local}, using HF hub: {self.model_path}")
        return self.model_path

    @staticmethod
    def _stack_incompatibilities() -> List[str]:
        """
        Report version mismatches that stop OpenVLA-7B from loading.

        OpenVLA's modeling code is pinned to a specific stack and calls
        APIs that were removed in later transformers (e.g.
        `tie_weights(recompute_mapping=...)`). Since the rest of ARIA
        needs the newer stack, the two cannot share an environment; we
        detect and report the mismatch instead of degrading silently.
        """
        issues: List[str] = []
        try:
            import transformers
            if not transformers.__version__.startswith("4.40"):
                issues.append(
                    f"transformers=={transformers.__version__} "
                    f"(OpenVLA requires {_OPENVLA_TRANSFORMERS_SPEC})")
        except ImportError:
            issues.append("transformers not installed")
        try:
            import timm
            if timm.__version__ not in _OPENVLA_TIMM_VERSIONS:
                issues.append(
                    f"timm=={timm.__version__} (OpenVLA requires one of "
                    f"{sorted(_OPENVLA_TIMM_VERSIONS)})")
        except ImportError:
            issues.append("timm not installed")
        return issues

    def load(self) -> bool:
        try:
            try:
                from transformers import AutoModelForVision2Seq, AutoProcessor
            except ImportError:
                from transformers import AutoModel as AutoModelForVision2Seq, AutoProcessor
            import torch

            issues = self._stack_incompatibilities()
            if issues and os.environ.get("ARIA_OPENVLA_FORCE_LOAD") != "1":
                print("[OpenVLA] This environment cannot run OpenVLA-7B as-is:")
                for issue in issues:
                    print(f"[OpenVLA]   - {issue}")
                print(
                    "[OpenVLA] OpenVLA is pinned to transformers=="
                    f"{_OPENVLA_TRANSFORMERS_SPEC}; the installed stack "
                    "(transformers 5.x / timm 1.x) is required by lerobot and "
                    "the rest of ARIA and cannot be downgraded in place. Run "
                    "OpenVLA in a dedicated virtual environment with the "
                    "pinned versions. Reporting OpenVLA as UNAVAILABLE "
                    "instead of producing fabricated actions.")
                self.loaded = False
                return False

            resolved_path = self._resolve_model_path()

            print(f"[OpenVLA] Loading processor from {resolved_path}...")
            self.processor = AutoProcessor.from_pretrained(
                resolved_path,
                trust_remote_code=True,
            )

            print("[OpenVLA] Loading model weights...")
            try:
                from transformers.dynamic_module_utils import get_class_from_dynamic_module
                model_cls = get_class_from_dynamic_module(
                    "modeling_prismatic.OpenVLAForActionPrediction", resolved_path)
            except Exception:
                try:
                    from transformers import AutoModelForVision2Seq as model_cls
                except ImportError:
                    from transformers import AutoModel as model_cls

            # Try 8-bit quantization first to fit in 8GB VRAM.
            # attn_implementation='eager' is required: OpenVLA's remote
            # code predates the SDPA support check in modern transformers.
            try:
                from transformers import BitsAndBytesConfig
                bnb_config = BitsAndBytesConfig(load_in_8bit=True)
                self.model = model_cls.from_pretrained(
                    resolved_path,
                    quantization_config=bnb_config,
                    trust_remote_code=True,
                    device_map="auto",
                    attn_implementation="eager",
                    low_cpu_mem_usage=True,
                )
                print("[OpenVLA] Loaded in 8-bit quantization (VRAM-efficient)")
            except Exception as q_err:
                print(f"[OpenVLA] 8-bit quant failed ({q_err}), loading bfloat16...")
                self.model = model_cls.from_pretrained(
                    resolved_path,
                    torch_dtype=torch.bfloat16,
                    trust_remote_code=True,
                )
                if torch.cuda.is_available():
                    self.model = self.model.cuda()

            self.model.eval()
            self.loaded = True
            print("[OpenVLA] ✅ Model ready")
            return True

        except ImportError as e:
            print(f"[OpenVLA] Import error: {e}. "
                  "Run: pip install transformers bitsandbytes accelerate")
            self.loaded = False
            return False
        except Exception as e:
            print(f"[OpenVLA] Load failed: {e}")
            self.loaded = False
            return False

    def predict(self, vla_input: VLAInput) -> VLAOutput:
        if not self.loaded or self.model is None:
            return VLAOutput(confidence=0.0, reasoning="OpenVLA not loaded")

        def _infer():
            import torch
            from PIL import Image as PILImage

            # ── Image selection: WRIST CAMERA PRIMARY ─────────────
            # Wrist/gripper camera is the primary visual for ARIA.
            # OpenVLA's Prismatic encoder handles any camera viewpoint.
            if vla_input.wrist_camera_image is not None:
                pil_img = PILImage.fromarray(vla_input.wrist_camera_image)
                cam_note = "wrist_cam"
            elif vla_input.top_camera_image is not None:
                pil_img = PILImage.fromarray(vla_input.top_camera_image)
                cam_note = "top_cam_fallback"
            else:
                pil_img = PILImage.new("RGB", (640, 480), color=(30, 30, 30))
                cam_note = "blank_frame"

            # ── Prompt: OpenVLA instruction format ────────────────
            instruction = vla_input.instruction or "pick up the object"
            prompt = f"In: What action should the robot take to {instruction}?\nOut:"

            # ── Encode and run inference ───────────────────────────
            inputs = self.processor(prompt, pil_img).to(
                self.model.device, dtype=torch.bfloat16
            )

            with torch.no_grad():
                # Uses the checkpoint's own unnormalisation statistics.
                action = self.model.predict_action(
                    **inputs,
                    unnorm_key=self.unnorm_key,
                    do_sample=False,
                )

            return action, cam_note

        try:
            (action, cam_note) = self._time_inference(_infer)

            # ── Map action to ARIA joints ──────────────────────────
            action_np = np.array(action, dtype=np.float32)

            if len(action_np) == 7:
                # bridge_orig layout: [dx,dy,dz,droll,dpitch,dyaw,gripper].
                # Project the Cartesian delta into ARIA joint space with an
                # explicit differential-IK step (uncalibrated).
                joint_deltas, ok = eef_delta_to_joint_delta(
                    vla_input.joint_positions, action_np[:3])
                if not ok:
                    return VLAOutput(
                        confidence=0.0,
                        reasoning=(
                            "OpenVLA emitted a Cartesian delta but ARIA's "
                            "differential IK is unavailable (FK module not "
                            "found); refusing to treat Cartesian deltas as "
                            "joint commands"))
                gripper_cmd = float(np.clip(action_np[6], 0.0, 1.0))
                mapping = "differential-IK projection (uncalibrated)"
            elif len(action_np) in (5, 6):
                # Fine-tuned ARIA checkpoint emitting joint-space actions:
                # four arm joints (+ optional extra) and a trailing gripper.
                joint_deltas = action_np[:4]
                gripper_cmd = float(np.clip(action_np[-1], 0.0, 1.0))
                mapping = "direct joint-space action"
            elif len(action_np) == 4:
                joint_deltas = action_np
                gripper_cmd = 0.0
                mapping = "direct joint-space action (no gripper dim)"
            else:
                return VLAOutput(
                    confidence=0.0,
                    reasoning=(
                        "Unsupported VLA action dimension: expected 7 "
                        "(OpenVLA bridge layout), 4/5/6 (joint-space "
                        f"checkpoint), got {len(action_np)}"
                    ),
                )

            output = VLAOutput()
            output.joint_commands = joint_deltas
            output.gripper_command = gripper_cmd
            output.is_delta = True
            # OpenVLA exposes no calibrated success probability. Report
            # action availability only (1.0 = a valid action was produced),
            # never a fabricated confidence number.
            output.confidence = 1.0 if joint_deltas.size else 0.0
            output.confidence_is_calibrated = False
            output.reasoning = (
                f"OpenVLA-7B [{cam_note}]: instruction-conditioned policy, "
                f"unnorm={self.unnorm_key}, mapping={mapping}, "
                f"inference={self.avg_inference_ms:.0f}ms, "
                f"gripper={gripper_cmd:.2f} "
                f"(no calibrated confidence exposed by the policy)"
            )
            return output

        except Exception as e:
            return VLAOutput(
                confidence=0.0,
                reasoning=f"OpenVLA inference error: {e}"
            )

    def unload(self):
        self.model = None
        self.processor = None
        self.loaded = False
        import gc
        gc.collect()
        try:
            import torch
            torch.cuda.empty_cache()
        except Exception:
            pass


# ═══════════════════════════════════════════════════════════════
# Backend 3: Pi0 (via lerobot)
# ═══════════════════════════════════════════════════════════════

class Pi0Backend(_LerobotPolicyBackend):
    """
    Physical Intelligence Pi0 policy, loaded through lerobot.

    ARIA does not ship Pi0 weights. This backend therefore activates
    only when a real Pi0 checkpoint path is provided; with no checkpoint
    it reports itself unavailable instead of running random weights.
    """

    policy_name = "pi0"
    policy_label = "Pi0"


# ═══════════════════════════════════════════════════════════════
# Backend 4: Gr00t N1 (via lerobot)
# ═══════════════════════════════════════════════════════════════

class Gr00tN1Backend(_LerobotPolicyBackend):
    """
    NVIDIA Gr00t N1 policy, loaded through lerobot.

    ARIA does not ship Gr00t weights, and the Gr00t module bundled with
    the installed lerobot (0.4.4) is itself broken at import time. This
    backend therefore reports itself unavailable unless a real Gr00t
    checkpoint is supplied, in which case it attempts an honest load and
    surfaces the real error. It never fabricates an action.
    """

    policy_name = "groot"
    policy_label = "Gr00t-N1"


# ═══════════════════════════════════════════════════════════════
# Unified VLA Interface
# ═══════════════════════════════════════════════════════════════

class VLAInterface:
    """
    Unified VLA interface — routes to the selected backend.

    Usage:
      vla = VLAInterface()
      vla.select_backend("LeRobot-ACT")
      vla.load()
      output = vla.predict(vla_input)
    """

    BACKENDS = {
        "LeRobot-ACT": LeRobotBackend,
        "OpenVLA": OpenVLABackend,
        "Pi0": Pi0Backend,
        "Gr00t-N1": Gr00tN1Backend,
    }

    def __init__(self):
        self.backends: Dict[str, VLABackend] = {}
        self.active_backend: Optional[VLABackend] = None
        self.active_name: str = ""

        # Register all backends
        for name, cls in self.BACKENDS.items():
            self.backends[name] = cls()

    def list_backends(self) -> List[str]:
        """List all available backend names."""
        return list(self.backends.keys())

    def select_backend(self, name: str) -> bool:
        """Select a backend by name."""
        if name not in self.backends:
            print(f"[VLA] Unknown backend: {name}. "
                  f"Available: {list(self.backends.keys())}")
            return False
        self.active_backend = self.backends[name]
        self.active_name = name
        return True

    def load(self, model_path: str = "") -> bool:
        """Load the selected backend's model."""
        if self.active_backend is None:
            print("[VLA] No backend selected")
            return False
        if model_path:
            self.active_backend.model_path = model_path
        return self.active_backend.load()

    def predict(self, vla_input: VLAInput) -> VLAOutput:
        """Run inference on the selected backend."""
        if self.active_backend is None or not self.active_backend.loaded:
            return VLAOutput(confidence=0.0, reasoning="No backend loaded")
        return self.active_backend.predict(vla_input)

    def unload(self):
        """Unload current backend."""
        if self.active_backend:
            self.active_backend.unload()

    def get_stats(self) -> dict:
        """Get performance stats for the active backend."""
        if self.active_backend is None:
            return {}
        return {
            "name": self.active_name,
            "loaded": self.active_backend.loaded,
            "avg_inference_ms": self.active_backend.avg_inference_ms,
            "total_inferences": len(self.active_backend.inference_times),
        }
