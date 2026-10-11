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
from typing import List, Tuple, Dict, Any

WORKSPACE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SCRIPTS_DIR = os.path.join(WORKSPACE_ROOT, "scripts")

# Outcome column identifiers that MUST NOT be assigned via random sampling or sleep
OUTCOME_KEYWORDS = {
    "success", "error", "latency", "force", "offset", "failure", "fail",
    "contact", "err_mm", "rot_err", "dt_ms", "hold_time", "displacement",
    "f_left", "f_right", "lift_success", "capture_success", "rebound",
    "uncomp_offset_mm", "comp_offset_mm", "pos_err_mm", "rot_err_rad"
}

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

    def visit_Assign(self, node: ast.Assign):
        # Extract target variable names
        assigned_names = []
        for target in node.targets:
            if isinstance(target, ast.Name):
                assigned_names.append(target.id)
            elif isinstance(target, ast.Subscript) and isinstance(target.slice, ast.Constant):
                if isinstance(target.slice.value, str):
                    assigned_names.append(target.slice.value)

        # Check if right-hand side contains random calls or time.sleep in outcome assignments
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


def checks_script(filepath: str) -> List[Tuple[int, str]]:
    with open(filepath, "r", encoding="utf-8") as f:
        content = f.read()

    # Only inspect scripts that write to or reference data/real
    if "data/real" not in content and "DATA_REAL_DIR" not in content and "data_real" not in content:
        return []

    try:
        tree = ast.parse(content, filename=filepath)
    except SyntaxError as e:
        return [(e.lineno or 0, f"SyntaxError parsing file: {e}")]

    visitor = SyntheticOutcomeVisitor(filepath)
    visitor.visit(tree)
    return visitor.violations


def main() -> int:
    print("=" * 80)
    print("PROJECT ARIA: SCIENTIFIC INTEGRITY LINTER (check_no_synthetic_outcomes.py)")
    print("Enforcing zero synthetic outcomes in data/real/ scripts.")
    print("=" * 80)

    total_checked = 0
    total_violations = 0
    all_violations: Dict[str, List[Tuple[int, str]]] = {}

    for root, _, files in os.walk(SCRIPTS_DIR):
        for file in sorted(files):
            if file.endswith(".py"):
                path = os.path.join(root, file)
                rel_path = os.path.relpath(path, WORKSPACE_ROOT)
                # Check if file targets data/real
                with open(path, "r", encoding="utf-8") as f:
                    c = f.read()
                if "data/real" in c or "DATA_REAL_DIR" in c:
                    total_checked += 1
                    violations = checks_script(path)
                    if violations:
                        all_violations[rel_path] = violations
                        total_violations += len(violations)
                        print(f"  [FAIL] {rel_path} ({len(violations)} violations)")
                    else:
                        print(f"  [PASS] {rel_path}")

    print("-" * 80)
    if total_violations == 0:
        print(f"SUCCESS: Checked {total_checked} active scripts targeting data/real/. Zero synthetic outcomes found.")
        print("=" * 80)
        return 0
    else:
        print(f"FAILED: Found {total_violations} synthetic outcome violation(s) across {len(all_violations)} file(s):")
        for fpath, viols in all_violations.items():
            print(f"\n  File: {fpath}")
            for lineno, msg in viols:
                print(f"    Line {lineno}: {msg}")
        print("=" * 80)
        return 1

if __name__ == "__main__":
    sys.exit(main())
