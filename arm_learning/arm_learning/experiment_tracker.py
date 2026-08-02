#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Experiment Tracker
Unified interface for W&B and MLflow experiment tracking.

Logs IK benchmarks, skill performance, training runs,
calibration quality, and session summaries.

Backend: "mlflow" (default, local) | "wandb" (cloud) | "none"
═══════════════════════════════════════════════════════════════
"""
import json
import os
import platform
import subprocess
import time
from dataclasses import dataclass, field, asdict
from datetime import datetime
from typing import Any, Dict, List, Optional

import yaml


# ═══════════════════════════════════════════════════════════════
# Data classes for structured metrics
# ═══════════════════════════════════════════════════════════════
@dataclass
class IKBenchmarkResults:
    solver_name: str
    n_test_poses: int
    success_rate: float              # 0.0 – 1.0
    mean_solve_time_ms: float
    p95_solve_time_ms: float
    position_error_mm: float
    orientation_error_deg: float = 0.0
    timestamp: str = ''

    def __post_init__(self):
        if not self.timestamp:
            self.timestamp = datetime.now().isoformat()


@dataclass
class SkillResult:
    skill_name: str
    success: bool
    duration_s: float
    attempts: int = 1
    object_class: str = ''
    material: str = ''
    task_id: str = ''
    failure_reason: str = ''
    timestamp: str = ''

    def __post_init__(self):
        if not self.timestamp:
            self.timestamp = datetime.now().isoformat()


@dataclass
class SessionMetrics:
    session_id: str
    start_time: str
    end_time: str
    mode: str = 'sim'                 # sim | hardware
    tasks_attempted: int = 0
    tasks_completed: int = 0
    overall_success_rate: float = 0.0
    total_grasps: int = 0
    successful_grasps: int = 0
    top_failures: List[str] = field(default_factory=list)
    objects_handled: List[str] = field(default_factory=list)
    git_sha: str = ''


@dataclass
class ComparisonReport:
    run_ids: List[str]
    metrics: Dict[str, Dict[str, float]]
    best_run: str = ''
    summary: str = ''


# ═══════════════════════════════════════════════════════════════
# Configuration
# ═══════════════════════════════════════════════════════════════
_DEFAULT_CONFIG_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'config', 'tracking_config.yaml')


def load_config(config_path: str = '') -> dict:
    """Load tracking configuration from YAML."""
    path = config_path or _DEFAULT_CONFIG_PATH
    defaults = {
        'backend': 'mlflow',
        'wandb_api_key': '',
        'mlflow_uri': 'http://localhost:5000',
        'log_every_skill': True,
        'log_every_ik_solve': False,
        'log_images': True,
        'auto_log_session': True,
        'project_name': 'aria',
    }
    if os.path.exists(path):
        try:
            with open(path, 'r') as f:
                loaded = yaml.safe_load(f)
            if loaded:
                defaults.update(loaded)
        except Exception:
            pass
    return defaults


# ═══════════════════════════════════════════════════════════════
# System info collector
# ═══════════════════════════════════════════════════════════════
def get_system_config() -> dict:
    """Collect system configuration for experiment metadata."""
    config = {
        'hostname': platform.node(),
        'os': f"{platform.system()} {platform.release()}",
        'python': platform.python_version(),
        'cpu': platform.processor() or 'unknown',
    }

    # GPU info
    try:
        result = subprocess.run(
            ['nvidia-smi', '--query-gpu=name,memory.total',
             '--format=csv,noheader'],
            capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            config['gpu'] = result.stdout.strip()
    except (FileNotFoundError, subprocess.TimeoutExpired):
        config['gpu'] = 'none'

    # Git SHA
    try:
        result = subprocess.run(
            ['git', 'rev-parse', '--short', 'HEAD'],
            capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            config['git_sha'] = result.stdout.strip()
    except (FileNotFoundError, subprocess.TimeoutExpired):
        config['git_sha'] = 'unknown'

    # ROS distro
    config['ros_distro'] = os.environ.get('ROS_DISTRO', 'unknown')

    return config


# ═══════════════════════════════════════════════════════════════
# Backend: MLflow
# ═══════════════════════════════════════════════════════════════
class MLflowBackend:
    """MLflow experiment tracking backend (fully local, no account)."""

    def __init__(self, tracking_uri: str, project: str):
        self.tracking_uri = tracking_uri
        self.project = project
        self._mlflow = None
        self._active_run = None

    def _ensure_mlflow(self):
        if self._mlflow is None:
            import mlflow
            mlflow.set_tracking_uri(self.tracking_uri)
            mlflow.set_experiment(self.project)
            self._mlflow = mlflow

    def start_run(self, run_name: str, tags: dict = None) -> str:
        self._ensure_mlflow()
        self._active_run = self._mlflow.start_run(
            run_name=run_name, tags=tags or {})
        return self._active_run.info.run_id

    def end_run(self):
        if self._mlflow and self._active_run:
            self._mlflow.end_run()
            self._active_run = None

    def log_params(self, params: dict):
        self._ensure_mlflow()
        for k, v in params.items():
            self._mlflow.log_param(k, v)

    def log_metrics(self, metrics: dict, step: int = 0):
        self._ensure_mlflow()
        for k, v in metrics.items():
            if isinstance(v, (int, float)):
                self._mlflow.log_metric(k, v, step=step)

    def log_artifact(self, filepath: str):
        self._ensure_mlflow()
        if os.path.exists(filepath):
            self._mlflow.log_artifact(filepath)


# ═══════════════════════════════════════════════════════════════
# Backend: W&B
# ═══════════════════════════════════════════════════════════════
class WandbBackend:
    """Weights & Biases experiment tracking backend (cloud)."""

    def __init__(self, project: str, api_key: str = ''):
        self.project = project
        self._wandb = None
        self._run = None
        if api_key:
            os.environ['WANDB_API_KEY'] = api_key

    def _ensure_wandb(self):
        if self._wandb is None:
            import wandb
            self._wandb = wandb

    def start_run(self, run_name: str, tags: dict = None) -> str:
        self._ensure_wandb()
        config = get_system_config()
        if tags:
            config.update(tags)
        self._run = self._wandb.init(
            project=self.project, name=run_name, config=config)
        return self._run.id

    def end_run(self):
        if self._wandb and self._run:
            self._wandb.finish()
            self._run = None

    def log_params(self, params: dict):
        if self._run:
            self._run.config.update(params)

    def log_metrics(self, metrics: dict, step: int = 0):
        if self._run:
            self._run.log(metrics, step=step)

    def log_artifact(self, filepath: str):
        if self._run and os.path.exists(filepath):
            artifact = self._wandb.Artifact(
                os.path.basename(filepath), type='result')
            artifact.add_file(filepath)
            self._run.log_artifact(artifact)


# ═══════════════════════════════════════════════════════════════
# Backend: None (offline fallback)
# ═══════════════════════════════════════════════════════════════
class NoneBackend:
    """No-op backend for offline work or when tracking is disabled."""

    def __init__(self):
        self._log_dir = os.path.expanduser('~/aria_experiments')
        os.makedirs(self._log_dir, exist_ok=True)
        self._current_log = None

    def start_run(self, run_name: str, tags: dict = None) -> str:
        run_id = f"{run_name}_{int(time.time())}"
        self._current_log = os.path.join(self._log_dir, f"{run_id}.json")
        with open(self._current_log, 'w') as f:
            json.dump({'run_name': run_name, 'tags': tags or {},
                       'metrics': {}, 'params': {}}, f, indent=2)
        return run_id

    def end_run(self):
        self._current_log = None

    def log_params(self, params: dict):
        self._append_to_log('params', params)

    def log_metrics(self, metrics: dict, step: int = 0):
        self._append_to_log('metrics', metrics)

    def log_artifact(self, filepath: str):
        pass

    def _append_to_log(self, section: str, data: dict):
        if not self._current_log or not os.path.exists(self._current_log):
            return
        try:
            with open(self._current_log, 'r') as f:
                log = json.load(f)
            log[section].update(data)
            with open(self._current_log, 'w') as f:
                json.dump(log, f, indent=2)
        except Exception:
            pass


# ═══════════════════════════════════════════════════════════════
# Main Tracker
# ═══════════════════════════════════════════════════════════════
class ExperimentTracker:
    """
    Unified experiment tracking interface.

    Usage:
        tracker = ExperimentTracker()  # reads config
        tracker.log_ik_benchmark(results)
        tracker.log_skill_performance('pick_up', result)
        tracker.log_session_summary(session)
    """

    def __init__(self, config_path: str = '', backend: str = ''):
        self.config = load_config(config_path)
        backend_name = backend or self.config.get('backend', 'none')

        if backend_name == 'mlflow':
            uri = self.config.get('mlflow_uri', 'http://localhost:5000')
            project = self.config.get('project_name', 'aria')
            self._backend = MLflowBackend(uri, project)
        elif backend_name == 'wandb':
            project = self.config.get('project_name', 'aria')
            api_key = self.config.get('wandb_api_key', '')
            self._backend = WandbBackend(project, api_key)
        else:
            self._backend = NoneBackend()

        self._backend_name = backend_name
        self._session_start = None

    @property
    def backend_name(self) -> str:
        return self._backend_name

    # ── IK Benchmarks ──────────────────────────────────────
    def log_ik_benchmark(self, results: IKBenchmarkResults):
        """Log IK benchmark run. Each solver becomes a separate run."""
        run_id = self._backend.start_run(
            f"ik_benchmark_{results.solver_name}",
            tags={'type': 'ik_benchmark', 'solver': results.solver_name})

        self._backend.log_params({
            'solver_name': results.solver_name,
            'n_test_poses': results.n_test_poses,
            'timestamp': results.timestamp,
        })

        self._backend.log_metrics({
            'ik_success_rate': results.success_rate,
            'ik_mean_solve_time_ms': results.mean_solve_time_ms,
            'ik_p95_solve_time_ms': results.p95_solve_time_ms,
            'ik_position_error_mm': results.position_error_mm,
            'ik_orientation_error_deg': results.orientation_error_deg,
        })

        self._backend.end_run()
        return run_id

    # ── Skill Performance ──────────────────────────────────
    def log_skill_performance(self, skill: str, result: SkillResult):
        """Log every skill execution for trend analysis."""
        if not self.config.get('log_every_skill', True):
            return

        run_id = self._backend.start_run(
            f"skill_{skill}_{int(time.time())}",
            tags={'type': 'skill', 'skill': skill})

        self._backend.log_params({
            'skill_name': result.skill_name,
            'object_class': result.object_class,
            'material': result.material,
            'task_id': result.task_id,
        })

        self._backend.log_metrics({
            'skill_success': 1.0 if result.success else 0.0,
            'skill_duration_s': result.duration_s,
            'skill_attempts': float(result.attempts),
        })

        if result.failure_reason:
            self._backend.log_params({
                'failure_reason': result.failure_reason})

        self._backend.end_run()

    # ── Training Runs ──────────────────────────────────────
    def log_training_run(self, config: dict, metrics: dict):
        """Log VLA or model training run."""
        model_name = config.get('model_name', 'unknown')
        run_id = self._backend.start_run(
            f"train_{model_name}_{int(time.time())}",
            tags={'type': 'training', 'model': model_name})

        self._backend.log_params(config)
        self._backend.log_metrics(metrics)
        self._backend.end_run()

    # ── Calibration ────────────────────────────────────────
    def log_calibration(
        self, before_error_mm: float, after_error_mm: float
    ):
        """Track calibration quality over sessions."""
        run_id = self._backend.start_run(
            f"calibration_{int(time.time())}",
            tags={'type': 'calibration'})

        self._backend.log_metrics({
            'calibration_error_before_mm': before_error_mm,
            'calibration_error_after_mm': after_error_mm,
            'calibration_improvement_mm': before_error_mm - after_error_mm,
        })

        self._backend.end_run()

    # ── Session Summary ────────────────────────────────────
    def log_session_summary(self, session: SessionMetrics):
        """End-of-session summary."""
        run_id = self._backend.start_run(
            f"session_{session.session_id}",
            tags={'type': 'session', 'mode': session.mode})

        self._backend.log_params({
            'session_id': session.session_id,
            'mode': session.mode,
            'start_time': session.start_time,
            'end_time': session.end_time,
            'git_sha': session.git_sha,
        })

        self._backend.log_metrics({
            'tasks_attempted': float(session.tasks_attempted),
            'tasks_completed': float(session.tasks_completed),
            'task_success_rate': session.overall_success_rate,
            'total_grasps': float(session.total_grasps),
            'successful_grasps': float(session.successful_grasps),
        })

        self._backend.end_run()

    # ── Run Comparison ─────────────────────────────────────
    def compare_runs(self, run_ids: List[str]) -> ComparisonReport:
        """Compare multiple runs side-by-side."""
        report = ComparisonReport(run_ids=run_ids, metrics={})

        if self._backend_name == 'mlflow':
            try:
                import mlflow
                for run_id in run_ids:
                    run = mlflow.get_run(run_id)
                    report.metrics[run_id] = run.data.metrics
            except Exception:
                pass
        elif self._backend_name == 'none':
            report.summary = "Comparison not available in offline mode"

        return report


# ═══════════════════════════════════════════════════════════════
# Module-level singleton for easy import
# ═══════════════════════════════════════════════════════════════
_tracker: Optional[ExperimentTracker] = None


def get_tracker(config_path: str = '', backend: str = '') -> ExperimentTracker:
    """Get or create the global experiment tracker."""
    global _tracker
    if _tracker is None:
        _tracker = ExperimentTracker(config_path, backend)
    return _tracker
