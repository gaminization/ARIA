#!/usr/bin/env python3
"""
scripts/check_no_synthetic_outcomes.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Scientific Integrity Linter for Project ARIA

Enforces the strict empirical measurement rule:
- Any value in a results CSV (success, error, latency, force, offset, etc.)
  must come from a measured system: Gazebo state, ROS timestamps, or real
  model outputs.
- Seeded randomness is allowed ONLY to pick initial poses, spawn times, or
  prompt order, strictly inside one function named `sample_initial_conditions()`.
- Never sample, assign by formula, or hard-code an outcome column.
- Never use time.sleep in code that assigns an outcome column or represents a process.

Fails (exit code 1) if any script writing to `data/real/` uses np.random,
random.*, rng.* or time.sleep in code that assigns an outcome column.
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""

import os
import sys
import ast
from typing import List, Tuple, Dict, Any, Optional

WORKSPACE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SCRIPTS_DIR = os.path.join(WORKSPACE_ROOT, "scripts")

# Outcome column identifiers that MUST NOT be assigned via random sampling or sleep
OUTCOME_KEYWORDS = {
    "success", "error", "latency", "force", "offset", "failure", "fail",
    "contact", "err_mm", "rot_err", "dt_ms", "hold_time", "displacement",
    "f_left", "f_right", "lift_success", "capture_success", "rebound",
    "uncomp_offset_mm", "comp_offset_mm", "pos_err_mm", "rot_err_rad"
}

PRED_KEYWORDS = {"pred", "estimate", "prediction"}
GT_KEYWORDS = {"gt", "ground_truth", "gt_x", "gt_y", "gt_z", "true_x", "true_y", "true_z"}
ERROR_KEYWORDS = {"err", "error", "diff", "residual", "loss"}

RANDOM_METHODS = {
    "random", "randint", "uniform", "normal", "choice", "choices",
    "sample", "shuffle", "gauss", "exponential", "beta", "rand", "randn",
    "standard_normal", "poisson", "binomial", "triangular", "vonmises"
}

class SyntheticOutcomeVisitor(ast.NodeVisitor):
    def __init__(self, filename: str):
        self.filename = filename
        self.violations: List[Tuple[int, str]] = []
        self.current_function: str = ""

    def is_outcome_name(self, name: str) -> bool:
        lower = name.lower()
        return any(k in lower for k in OUTCOME_KEYWORDS)

    def is_pred_name(self, name: str) -> bool:
        lower = name.lower()
        has_pred = any(k in lower for k in PRED_KEYWORDS)
        has_err = any(k in lower for k in ERROR_KEYWORDS)
        return has_pred and not has_err

    def is_gt_name(self, name: str) -> bool:
        lower = name.lower()
        return any(k in lower for k in GT_KEYWORDS)

    def contains_gt_reference(self, node: ast.AST) -> Optional[str]:
        for child in ast.walk(node):
            if isinstance(child, ast.Name) and self.is_gt_name(child.id):
                return child.id
        return None

    def is_detection_test(self, test_node: ast.AST) -> bool:
        test_str = ast.unparse(test_node).lower()
        return any(k in test_str for k in ["boxes", "detector", "detect", "conf", "bbox"])

    def is_random_call(self, node: ast.AST) -> bool:
        """Detects calls to np.random.*, random.*, rng.*"""
        if not isinstance(node, ast.Call):
            return False

        func = node.func
        # e.g. np.random.uniform(...) or rng.uniform(...) or random.gauss(...)
        if isinstance(func, ast.Attribute):
            val = func.value
            # case 1: np.random.<attr> or numpy.random.<attr>
            if isinstance(val, ast.Attribute) and isinstance(val.value, ast.Name):
                if val.value.id in {"np", "numpy"} and val.attr == "random":
                    return True
            # case 2: random.<attr>
            if isinstance(val, ast.Name) and val.id == "random":
                return True
            # case 3: rng.<attr>
            if isinstance(val, ast.Name) and (val.id == "rng" or "rng" in val.id.lower()):
                if func.attr in RANDOM_METHODS:
                    return True
            # case 4: np.random(..)
            if isinstance(val, ast.Name) and val.id in {"np", "numpy"} and func.attr == "random":
                return True
        elif isinstance(func, ast.Name):
            # bare call if imported `from random import uniform, ...`
            if func.id in RANDOM_METHODS:
                return True

        return False

    def is_sleep_call(self, node: ast.AST) -> bool:
        """Detects calls to time.sleep(...)"""
        if isinstance(node, ast.Call):
            func = node.func
            if isinstance(func, ast.Attribute) and func.attr == "sleep":
                if isinstance(func.value, ast.Name) and func.value.id == "time":
                    return True
            elif isinstance(func, ast.Name) and func.id == "sleep":
                return True
        return False

    def visit_FunctionDef(self, node: ast.FunctionDef):
        prev_fn = self.current_function
        self.current_function = node.name
        self.generic_visit(node)
        self.current_function = prev_fn

    def visit_AsyncFunctionDef(self, node: ast.AsyncFunctionDef):
        prev_fn = self.current_function
        self.current_function = node.name
        self.generic_visit(node)
        self.current_function = prev_fn

    def visit_If(self, node: ast.If):
        # Visit the test and body normally
        self.visit(node.test)
        for stmt in node.body:
            self.visit(stmt)

        # If the if condition was testing for detection presence, inspect orelse for illicit fallbacks
        if self.is_detection_test(node.test) and node.orelse:
            prev_fallback = getattr(self, "in_detection_fallback", False)
            self.in_detection_fallback = True
            for stmt in node.orelse:
                self.visit(stmt)
            self.in_detection_fallback = prev_fallback
        else:
            for stmt in node.orelse:
                self.visit(stmt)

    def visit_Assign(self, node: ast.Assign):
        # Extract target variable names
        assigned_names = []
        for target in node.targets:
            if isinstance(target, ast.Name):
                assigned_names.append(target.id)
            elif isinstance(target, ast.Subscript) and isinstance(target.slice, ast.Constant):
                if isinstance(target.slice.value, str):
                    assigned_names.append(target.slice.value)

        # Rule 1: Prediction variable assigned from ground-truth source
        for name in assigned_names:
            if self.is_pred_name(name):
                gt_ref = self.contains_gt_reference(node.value)
                if gt_ref:
                    self.violations.append((
                        node.lineno,
                        f"Prediction variable '{name}' assigned from ground-truth source '{gt_ref}': {ast.unparse(node.value)}"
                    ))
                    break

        # Rule 2: Illicit fallback on failed detection (assigning error=0 or non-NaN pred)
        if getattr(self, "in_detection_fallback", False):
            for name in assigned_names:
                lower = name.lower()
                # If error is assigned to 0 or 0.0 in detection fallback
                if any(k in lower for k in ERROR_KEYWORDS):
                    if isinstance(node.value, ast.Constant) and node.value.value in (0, 0.0):
                        self.violations.append((
                            node.lineno,
                            f"Detection failure fallback sets error to zero instead of NaN: {ast.unparse(node.value)}"
                        ))
                # If prediction is assigned in detection fallback to something other than nan/none
                if self.is_pred_name(name):
                    val_str = ast.unparse(node.value).lower()
                    if "nan" not in val_str and "none" not in val_str:
                        self.violations.append((
                            node.lineno,
                            f"Detection failure fallback assigns synthetic non-NaN prediction '{name}': {ast.unparse(node.value)}"
                        ))

        # Rule 3: Success flag ignoring failed detection (in perception/vision scripts)
        if getattr(self, "has_detector_in_file", False):
            for name in assigned_names:
                if name.lower() in {"success", "succ"}:
                    val_str = ast.unparse(node.value).lower()
                    if "err" in val_str and ("<=" in val_str or "<" in val_str):
                        has_det_check = any(k in val_str for k in ["detect", "box", "conf", "isnan", "is not none", "valid"])
                        if not has_det_check:
                            self.violations.append((
                                node.lineno,
                                f"Success flag ignores detection status (evaluates error without requiring positive detection or checking NaN): {ast.unparse(node.value)}"
                            ))

        # Rule 4: Check if right-hand side contains random calls or time.sleep in outcome assignments
        for child in ast.walk(node.value):
            if self.is_random_call(child):
                # If outside sample_initial_conditions and assigns an outcome variable
                for name in assigned_names:
                    if self.is_outcome_name(name) and self.current_function != "sample_initial_conditions":
                        self.violations.append((
                            node.lineno,
                            f"Outcome variable '{name}' assigned via synthetic randomness: {ast.unparse(node.value)}"
                        ))
                        break

            if self.is_sleep_call(child):
                for name in assigned_names:
                    if self.is_outcome_name(name):
                        self.violations.append((
                            node.lineno,
                            f"Outcome variable '{name}' assigned using time.sleep"
                        ))
                        break

        self.generic_visit(node)

    def visit_Dict(self, node: ast.Dict):
        # Catch CSV rows constructed as dicts: {"success": ..., "latency_ms": ...}
        for k, v in zip(node.keys, node.values):
            if k is not None and isinstance(k, ast.Constant) and isinstance(k.value, str):
                key_name = k.value

                # Rule 1 in Dict: prediction column assigned from ground truth
                if self.is_pred_name(key_name):
                    gt_ref = self.contains_gt_reference(v)
                    if gt_ref:
                        self.violations.append((
                            node.lineno,
                            f"CSV prediction column '{key_name}' assigned from ground-truth source '{gt_ref}': {ast.unparse(v)}"
                        ))

                if self.is_outcome_name(key_name):
                    for child in ast.walk(v):
                        if self.is_random_call(child):
                            if self.current_function != "sample_initial_conditions":
                                self.violations.append((
                                    node.lineno,
                                    f"CSV outcome column '{key_name}' assigned via synthetic randomness: {ast.unparse(v)}"
                                ))
                        if self.is_sleep_call(child):
                            self.violations.append((
                                node.lineno,
                                f"CSV outcome column '{key_name}' assigned using time.sleep: {ast.unparse(v)}"
                            ))
        self.generic_visit(node)


def checks_script(filepath: str, require_data_real: bool = True) -> List[Tuple[int, str]]:
    with open(filepath, "r", encoding="utf-8") as f:
        content = f.read()

    # Historically only scripts writing to `data/real/` were inspected.
    # The linter now also covers the ARM packages and benchmark scripts,
    # so callers may pass require_data_real=False.
    if require_data_real:
        if ("data/real" not in content and "DATA_REAL_DIR" not in content
                and "data_real" not in content):
            return []

    try:
        tree = ast.parse(content, filename=filepath)
    except SyntaxError as e:
        return [(e.lineno or 0, f"SyntaxError parsing file: {e}")]

    visitor = SyntheticOutcomeVisitor(filepath)
    c_lower = content.lower()
    visitor.has_detector_in_file = bool(
        "detector" in c_lower or "boxes" in c_lower or "yolo" in c_lower or "predict" in c_lower
    )
    visitor.visit(tree)
    return visitor.violations


# Directories covered by the wider sweep (package source + all scripts).
SCAN_DIRS = [
    "scripts",
    "arm_vla",
    "arm_planner",
    "arm_agents",
    "arm_vision",
    "arm_control",
    "arm_ik",
    "arm_learning",
    "arm_dashboard",
    "arm_bringup",
    "arm_moveit_config",
]

_EXCLUDE_DIR_NAMES = {
    "build", "install", "third_party", "__pycache__", "node_modules",
    "legacy_synthetic", ".git",
}


def _iter_scan_files():
    """Yield package/benchmark Python files to inspect (deduplicated)."""
    seen = set()
    for base_rel in SCAN_DIRS:
        base = os.path.join(WORKSPACE_ROOT, base_rel)
        if not os.path.isdir(base):
            continue
        for root, dirs, files in os.walk(base):
            dirs[:] = [d for d in dirs if d not in _EXCLUDE_DIR_NAMES]
            for file in sorted(files):
                if not file.endswith(".py"):
                    continue
                if file.startswith("test_") or file.endswith("_test.py"):
                    continue
                path = os.path.abspath(os.path.join(root, file))
                if path in seen:
                    continue
                seen.add(path)
                yield path


def main() -> int:
    print("=" * 80)
    print("PROJECT ARIA: SCIENTIFIC INTEGRITY LINTER (check_no_synthetic_outcomes.py)")
    print("Enforcing zero synthetic outcomes in result-producing code")
    print("(scripts/ and all ARM package modules).")
    print("=" * 80)

    total_checked = 0
    total_violations = 0
    all_violations: Dict[str, List[Tuple[int, str]]] = {}

    for path in _iter_scan_files():
        rel_path = os.path.relpath(path, WORKSPACE_ROOT)
        total_checked += 1
        violations = checks_script(path, require_data_real=False)
        if violations:
            all_violations[rel_path] = violations
            total_violations += len(violations)
            print(f"  [FAIL] {rel_path} ({len(violations)} violations)")

    print("-" * 80)
    if total_violations == 0:
        print(f"SUCCESS: Checked {total_checked} source files. "
              "Zero synthetic outcomes found.")
        print("=" * 80)
        return 0
    else:
        print(f"FAILED: Found {total_violations} synthetic outcome violation(s) "
              f"across {len(all_violations)} file(s):")
        for fpath, viols in all_violations.items():
            print(f"\n  File: {fpath}")
            for lineno, msg in viols:
                print(f"    Line {lineno}: {msg}")
        print("=" * 80)
        return 1

if __name__ == "__main__":
    sys.exit(main())
