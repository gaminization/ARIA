#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Skill Manager
Registry, dispatcher, versioning, and rollback for all skills.
═══════════════════════════════════════════════════════════════
"""
import time
from typing import Any, Callable, Dict, List, Optional
from dataclasses import dataclass, field
from collections import defaultdict

import numpy as np


@dataclass
class SkillResult:
    """Result of a skill execution."""
    success: bool
    duration_s: float
    details: str = ""
    error: str = ""
    metrics: dict = field(default_factory=dict)


@dataclass
class PerformanceStats:
    """Aggregate performance stats for a skill."""
    name: str
    attempts: int = 0
    successes: int = 0
    failures: int = 0
    total_duration: float = 0.0
    recent_results: List[bool] = field(default_factory=list)

    @property
    def success_rate(self) -> float:
        return self.successes / max(self.attempts, 1)

    @property
    def avg_duration(self) -> float:
        return self.total_duration / max(self.attempts, 1)

    @property
    def recent_rate(self) -> float:
        """Success rate of last 20 attempts."""
        recent = self.recent_results[-20:]
        return sum(recent) / max(len(recent), 1)

    def summary(self) -> str:
        return (
            f"{self.name}: {self.success_rate:.0%} success "
            f"({self.successes}/{self.attempts}), "
            f"avg {self.avg_duration:.2f}s, "
            f"recent {self.recent_rate:.0%}")


@dataclass
class SkillVersion:
    """A versioned skill implementation."""
    version: str
    impl: Callable
    registered_at: float = 0.0
    stats: PerformanceStats = field(default_factory=lambda: PerformanceStats(""))


class SkillManager:
    """
    Registry and dispatcher for all manipulation skills.

    Features:
      - Register/unregister skills by name
      - Versioned skill implementations
      - Performance tracking per skill
      - Automatic rollback if new version performs worse
      - Extended skills: in-hand manipulation operations

    Built-in skills: pick, place, push, pull, stack, sort,
                     inspect, slide, roll, sweep
    Extended skills: rotate_in_hand, reposition_grip, flip, slide_to_tip
    """

    BUILTIN_SKILLS = [
        'pick', 'place', 'push', 'pull', 'stack',
        'sort', 'inspect', 'slide', 'roll', 'sweep',
    ]

    EXTENDED_SKILLS = [
        'rotate_in_hand', 'reposition_grip', 'flip', 'slide_to_tip',
    ]

    def __init__(self):
        self.skills: Dict[str, List[SkillVersion]] = {}
        self.active_versions: Dict[str, int] = {}  # name → version index
        self.global_stats: Dict[str, PerformanceStats] = {}
        self.execution_log: list = []

        # Register builtin placeholders
        for name in self.BUILTIN_SKILLS + self.EXTENDED_SKILLS:
            self.register_skill(name, self._placeholder_skill, "1.0.0")

    def register_skill(self, name: str, impl: Callable,
                       version: str = "1.0.0"):
        """
        Register a skill implementation.

        Args:
            name: skill name (e.g. 'pick')
            impl: callable(kwargs) → SkillResult
            version: semantic version string
        """
        if name not in self.skills:
            self.skills[name] = []
            self.active_versions[name] = 0
            self.global_stats[name] = PerformanceStats(name=name)

        sv = SkillVersion(
            version=version,
            impl=impl,
            registered_at=time.time(),
            stats=PerformanceStats(name=f"{name}@{version}"),
        )
        self.skills[name].append(sv)
        self.active_versions[name] = len(self.skills[name]) - 1

    def execute_skill(self, name: str, **kwargs) -> SkillResult:
        """
        Execute a named skill.

        Args:
            name: skill name
            **kwargs: skill-specific parameters

        Returns:
            SkillResult with success, duration, details
        """
        if name not in self.skills or not self.skills[name]:
            return SkillResult(
                success=False, duration_s=0.0,
                error=f"Unknown skill: {name}")

        active_idx = self.active_versions.get(name, 0)
        sv = self.skills[name][active_idx]

        t0 = time.time()
        try:
            result = sv.impl(**kwargs)
            if not isinstance(result, SkillResult):
                result = SkillResult(
                    success=bool(result),
                    duration_s=time.time() - t0)
        except Exception as e:
            result = SkillResult(
                success=False,
                duration_s=time.time() - t0,
                error=str(e))

        result.duration_s = time.time() - t0

        # Update stats
        self._update_stats(name, sv, result)

        # Auto-rollback check
        self._check_rollback(name)

        return result

    def get_performance(self, name: str) -> Optional[PerformanceStats]:
        """Get aggregate performance stats for a skill."""
        return self.global_stats.get(name)

    def get_all_performance(self) -> Dict[str, PerformanceStats]:
        """Get performance for all skills."""
        return dict(self.global_stats)

    def rollback_skill(self, name: str, to_version: str = ""):
        """
        Rollback a skill to a previous version.

        If to_version is empty, rolls back to the previous version.
        """
        if name not in self.skills:
            return False

        versions = self.skills[name]
        current_idx = self.active_versions.get(name, 0)

        if to_version:
            # Find specific version
            for i, sv in enumerate(versions):
                if sv.version == to_version:
                    self.active_versions[name] = i
                    return True
            return False
        else:
            # Roll back one version
            if current_idx > 0:
                self.active_versions[name] = current_idx - 1
                return True
            return False

    def list_skills(self) -> List[dict]:
        """List all registered skills with versions and stats."""
        result = []
        for name in sorted(self.skills.keys()):
            versions = self.skills[name]
            active_idx = self.active_versions.get(name, 0)
            stats = self.global_stats.get(name, PerformanceStats(name=name))
            result.append({
                'name': name,
                'active_version': versions[active_idx].version if versions else '',
                'n_versions': len(versions),
                'attempts': stats.attempts,
                'success_rate': stats.success_rate,
                'avg_duration': stats.avg_duration,
                'is_builtin': name in self.BUILTIN_SKILLS,
                'is_extended': name in self.EXTENDED_SKILLS,
            })
        return result

    # ── Internal ───────────────────────────────────────────

    def _update_stats(self, name: str, sv: SkillVersion,
                      result: SkillResult):
        """Update performance stats after execution."""
        # Per-version stats
        sv.stats.attempts += 1
        sv.stats.total_duration += result.duration_s
        sv.stats.recent_results.append(result.success)
        if result.success:
            sv.stats.successes += 1
        else:
            sv.stats.failures += 1

        # Global stats
        gs = self.global_stats[name]
        gs.attempts += 1
        gs.total_duration += result.duration_s
        gs.recent_results.append(result.success)
        if result.success:
            gs.successes += 1
        else:
            gs.failures += 1

        # Trim recent results
        if len(gs.recent_results) > 100:
            gs.recent_results = gs.recent_results[-50:]
        if len(sv.stats.recent_results) > 100:
            sv.stats.recent_results = sv.stats.recent_results[-50:]

        # Execution log
        self.execution_log.append({
            'skill': name,
            'version': sv.version,
            'success': result.success,
            'duration_s': result.duration_s,
            'time': time.time(),
        })
        if len(self.execution_log) > 1000:
            self.execution_log = self.execution_log[-500:]

    def _check_rollback(self, name: str):
        """
        Auto-rollback if new version performs significantly worse.

        Condition: ≥10 attempts AND recent_rate < 50% AND previous
                   version had recent_rate > 70%
        """
        versions = self.skills.get(name, [])
        active_idx = self.active_versions.get(name, 0)

        if active_idx == 0 or len(versions) < 2:
            return  # Nothing to roll back to

        current = versions[active_idx]
        previous = versions[active_idx - 1]

        if (current.stats.attempts >= 10 and
                current.stats.recent_rate < 0.50 and
                previous.stats.recent_rate > 0.70):
            self.active_versions[name] = active_idx - 1
            # Log rollback
            self.execution_log.append({
                'skill': name,
                'event': 'AUTO_ROLLBACK',
                'from_version': current.version,
                'to_version': previous.version,
                'reason': (
                    f"Current {current.stats.recent_rate:.0%} < 50%, "
                    f"previous {previous.stats.recent_rate:.0%} > 70%"),
                'time': time.time(),
            })

    def _placeholder_skill(self, **kwargs) -> SkillResult:
        """Placeholder for unimplemented skills."""
        return SkillResult(
            success=True,
            duration_s=0.1,
            details="Placeholder execution (no real hardware)")
