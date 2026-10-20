#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Deterministic Intent Planner
═══════════════════════════════════════════════════════════════
Fast, bounded-domain natural-language → skill-plan compiler.

Why this exists
---------------
The LLM planner is excellent at open-ended language but its latency is
unbounded and, on CPU-only hosts, dominates the control loop. ARIA's
onboard vocabulary is finite (the skill registry), so every common
operator request can be compiled deterministically in microseconds.

This is NOT a hardcoded outcome table: it only decides *which skills to
run and on which real entities*. Execution, perception, IK, grasping and
verification remain fully physical and are never fabricated here.

The compiler returns a plan dict compatible with
LLMPlanningAgent._apply_plan_to_task:

    {
      "goal": str,
      "subgoals": [str],
      "actions": [{"skill": str, "parameters": dict, "reasoning": str}],
      "confidence": float,
      "requires_approval": bool,
    }

Unknown / ambiguous commands return None so the caller can fall back to
the LLM (if allowed) or the generic rule-based decomposer.
"""

from __future__ import annotations

import re
from typing import Any, Dict, List, Optional


# ──────────────────────────────────────────────────────────────
# Vocabulary helpers
# ──────────────────────────────────────────────────────────────
_STOP = {
    "the", "a", "an", "this", "that", "these", "those", "please",
    "can", "you", "could", "would", "now", "then", "and", "to",
    "for", "me", "it", "its", "my",
}

_COLOR_WORDS = {
    "red", "blue", "green", "yellow", "orange", "white", "black",
    "brown", "wood", "wooden", "cyan", "purple", "pink", "grey", "gray",
}

_COLOR_ALIASES = {
    "wooden": "wood", "brown": "wood", "gray": "grey",
}

# Object nouns → canonical search phrase. The skill layer matches against
# live world-model class names, so these are search hints, not fixtures.
_OBJECT_ALIASES = {
    "orange ball": "orange",
    "orange fruit": "orange",
    "ball": "orange",
    "banana": "banana",
    "duck": "duck",
    "rubber duck": "duck",
    "mug": "mug",
    "cup": "mug",
    "tea mug": "mug",
    "coffee mug": "mug",
    "glass": "plastic_glass",
    "clear glass": "plastic_glass",
    "tumbler": "plastic_glass",
    "bottle": "bottle",
    "beverage bottle": "bottle",
    "bowl": "bowl",
    "plate": "dinner_plate",
    "dinner plate": "dinner_plate",
    "pan": "tefal_pan",
    "frying pan": "tefal_pan",
    "tray": "sorting_tray",
    "sorting tray": "sorting_tray",
    "box": "box",
    "pop tarts box": "pop_tarts_box",
    "jenga tower": "jenga_tower",
    "tower": "jenga_tower",
    "jenga block": "jenga_block",
    "block": "jenga_block",
    "brick": "jenga_block",
    "workpiece": "workpiece",
    "cube": "workpiece",
    "part": "workpiece",
    "object": "object",
}

_RECEPTACLE_HINTS = {
    "pan", "plate", "bowl", "tray", "box", "bin", "glass", "cup",
    "mug", "basket", "container", "dish", "ground", "floor",
}

_SPEED_WORDS = {
    "slow": 25.0, "slowest": 15.0,
    "medium": 50.0, "mid": 50.0, "normal": 50.0,
    "fast": 75.0, "faster": 85.0, "full": 100.0, "max": 100.0,
    "maximum": 100.0, "highest": 100.0,
}


def _norm(text: str) -> str:
    return re.sub(r"\s+", " ", (text or "").lower().strip())


def _tokens(text: str) -> List[str]:
    return re.findall(r"[a-z0-9]+", _norm(text))


def _strip_stop(tokens: List[str]) -> List[str]:
    return [t for t in tokens if t not in _STOP]


def _canonical_object(phrase: str) -> str:
    """Map a free-form noun phrase to the best live-search phrase."""
    phrase = _norm(phrase)
    phrase = re.sub(r"^(the|a|an)\s+", "", phrase).strip()
    if not phrase:
        return ""
    # Direct alias, longest first so "orange ball" beats "ball".
    for alias in sorted(_OBJECT_ALIASES, key=len, reverse=True):
        if phrase == alias or phrase.endswith(" " + alias) or alias in phrase.split():
            # Preserve an explicit colour qualifier if the caller gave one.
            colors = [t for t in _tokens(phrase) if t in _COLOR_WORDS]
            base = _OBJECT_ALIASES[alias]
            if colors and base not in _COLOR_WORDS:
                return f"{colors[0]} {base}"
            return base
    # Keep multi-word descriptors (e.g. "blue cube") but drop stop words.
    kept = _strip_stop(_tokens(phrase))
    return " ".join(kept)


def _split_put(command: str) -> Optional[tuple]:
    """Split 'put X in/into/on Y' → (X, Y)."""
    m = re.search(
        r"\b(?:put|place|set|drop|insert|throw)\b\s+(.+?)\s+"
        r"\b(?:in|into|inside|on|onto|at|inside of)\b\s+(.+)$",
        command,
    )
    if not m:
        return None
    return m.group(1).strip(), m.group(2).strip()


def _extract_speed(command: str) -> Optional[float]:
    m = re.search(r"(\d+(?:\.\d+)?)\s*%?", command)
    if m:
        try:
            return max(0.0, min(100.0, float(m.group(1))))
        except ValueError:
            pass
    for word, value in _SPEED_WORDS.items():
        if re.search(rf"\b{word}\b", command):
            return value
    return None


def _actions(*specs) -> List[Dict[str, Any]]:
    out = []
    for spec in specs:
        if isinstance(spec, str):
            out.append({"skill": spec, "parameters": {}, "reasoning": ""})
        else:
            out.append(spec)
    return out


def _act(skill: str, reasoning: str = "", **parameters) -> Dict[str, Any]:
    return {"skill": skill, "parameters": parameters, "reasoning": reasoning}


class IntentPlanner:
    """Compile a command into a deterministic ARIA skill plan."""

    def __init__(self):
        # Ordered most-specific → least-specific. Each entry is
        # (compiler_name, predicate). Compilers return a plan or None.
        self._compilers = [
            self._compile_conveyor,
            self._compile_gesture,
            self._compile_sort_by_color,
            self._compile_pick_all,
            self._compile_stack_all,
            self._compile_drop,
            self._compile_nest_all,
            self._compile_stack,
            self._compile_put,
            self._compile_pick_handle,
            self._compile_press,
            self._compile_find,
            self._compile_pick,
            self._compile_place,
            self._compile_scan,
        ]

    # ── Public API ───────────────────────────────────────────
    def plan(self, command: str, world_objects: Optional[list] = None) -> Optional[Dict[str, Any]]:
        cmd = _norm(command)
        if not cmd:
            return None

        for compiler in self._compilers:
            try:
                plan = compiler(cmd, world_objects or [])
            except Exception:
                plan = None
            if plan:
                plan.setdefault("requires_approval", False)
                plan.setdefault("confidence", 0.9)
                plan.setdefault("subgoals", [
                    a["skill"] for a in plan.get("actions", [])
                ])
                return plan
        return None

    # ── Compilers ────────────────────────────────────────────
    def _compile_conveyor(self, cmd: str, objs: list) -> Optional[dict]:
        if not re.search(r"\b(conveyor|belt)\b", cmd):
            return None

        stop = bool(re.search(r"\b(stop|halt|pause|turn off|shut off)\b", cmd))
        start = bool(re.search(r"\b(start|run|begin|turn on|spin up|activate)\b", cmd))
        switch = bool(re.search(r"\b(switch|change|set|adjust|move)\b", cmd))
        all_speeds = bool(re.search(r"\b(all|every|each)\b.*\b(speed|speeds)\b", cmd)) \
            or bool(re.search(r"\bspeed(s)?\b.*\ball\b", cmd))

        if all_speeds:
            sweep = [15.0, 30.0, 50.0, 75.0, 100.0]
            actions = []
            for p in sweep:
                actions.append(_act(
                    "conveyor", f"Run belt at {p:.0f}% power", power=p))
                actions.append(_act("wait", f"Hold {p:.0f}% for 2 s", seconds=2.0))
            actions.append(_act("conveyor", "Stop belt", power=0.0))
            return {
                "goal": "Sweep the conveyor through every available speed",
                "actions": actions,
                "confidence": 0.95,
            }

        if stop:
            return {
                "goal": "Stop the conveyor belt",
                "actions": [_act("conveyor", "Stop belt", power=0.0)],
                "confidence": 0.97,
            }

        speed = _extract_speed(cmd)
        if speed is None:
            speed = 50.0
        verb = "Set" if switch else ("Start" if start else "Set")
        return {
            "goal": f"{verb} conveyor belt to {speed:.0f}% power",
            "actions": [
                _act("conveyor", f"{verb} belt at {speed:.0f}%", power=speed),
                _act("wait", "Allow the belt to settle", seconds=0.5),
            ],
            "confidence": 0.95,
        }

    def _compile_gesture(self, cmd: str, objs: list) -> Optional[dict]:
        wants_wave = bool(re.search(r"\b(wave|waving|say hello|greet)\b", cmd))
        wants_nod = bool(re.search(r"\b(nod|nodding|say yes)\b", cmd))
        wants_no = bool(re.search(
            r"\b(shake|say no|gesture no|head shake|disagree|deny)\b", cmd))
        wants_drink = bool(re.search(
            r"\b(drink|sip|coffee|tea|beverage)\b", cmd))

        if not (wants_wave or wants_nod or wants_no or wants_drink):
            return None

        # "pick up the mug and make a coffee drinking gesture"
        pick_phrase = None
        if wants_drink:
            m = re.search(
                r"\b(?:pick|grab|take|get|lift)\b(?:\s+up)?\s+(.+?)"
                r"(?:\s+and\b|\s+then\b|\s+to\b|$)", cmd)
            if m:
                pick_phrase = _canonical_object(m.group(1))

        actions = []
        if pick_phrase and pick_phrase not in {"object", ""}:
            actions.append(_act("pick", f"Grasp {pick_phrase} before the gesture",
                                object_id=pick_phrase))

        if wants_wave:
            actions.append(_act("joint_motion", "Wave the gripper", gesture="wave"))
            goal = "Wave at the camera"
        elif wants_nod:
            actions.append(_act("joint_motion", "Nod the wrist", gesture="nod"))
            goal = "Nod in agreement"
        elif wants_no:
            actions.append(_act("joint_motion", "Shake the wrist side to side",
                                gesture="no"))
            goal = "Gesture 'no'"
        else:
            actions.append(_act("joint_motion",
                                "Raise the grasped vessel to the mouth and tilt",
                                gesture="drink"))
            goal = ("Drink from the mug" if pick_phrase
                    else "Perform a coffee-drinking gesture")

        return {
            "goal": goal,
            "actions": actions,
            "confidence": 0.96,
        }

    def _compile_sort_by_color(self, cmd: str, objs: list) -> Optional[dict]:
        if not re.search(r"\bsort\b|\bgroup\b|\borgani[sz]e\b", cmd):
            return None
        if not re.search(r"\bcolou?r", cmd) and "sort" not in cmd:
            return None
        if not re.search(r"\ball\b|\bevery\b|\bobjects\b|\bitems\b|\bworkpieces\b", cmd):
            return None
        destination = "sorting_tray"
        m = re.search(r"\b(?:in|into|onto|on)\s+(?:the\s+)?(.+)$", cmd)
        if m:
            cand = _canonical_object(m.group(1))
            if cand:
                destination = cand
        return {
            "goal": f"Sort every object by colour into the {destination}",
            "actions": [
                _act("scan_workspace", "Refresh the world model before sorting"),
                _act("sort_by_color",
                     "Group live objects by detected colour and deposit each group",
                     destination=destination),
            ],
            "confidence": 0.93,
        }

    def _compile_pick_all(self, cmd: str, objs: list) -> Optional[dict]:
        if not re.search(r"\bpick\b|\bgrab\b|\blift\b|\btake\b", cmd):
            return None
        if not re.search(r"\b(every|all|each)\b", cmd):
            return None
        handle = bool(re.search(r"\bhandle", cmd))
        # "pick up every object available in the world"
        params: Dict[str, Any] = {}
        if handle:
            params["grasp_strategy"] = "handle"
        # Optional explicit class filter ("pick up all the mugs")
        m = re.search(
            r"\b(?:every|all|each)\s+(?:of\s+the\s+|the\s+)?([a-z ]+?)"
            r"(?:\s+available|\s+in the world|\s+on the table|$)", cmd)
        target = "all"
        if m:
            cand = _canonical_object(m.group(1))
            if cand and cand not in {"object", "all"}:
                target = cand
        params["object_id"] = target
        return {
            "goal": f"Pick up {target if target != 'all' else 'every reachable object'}",
            "actions": [
                _act("scan_workspace", "Refresh the world model"),
                _act("pick_all", "Pick and verify every requested object",
                     **params),
            ],
            "confidence": 0.9,
        }

    def _compile_stack_all(self, cmd: str, objs: list) -> Optional[dict]:
        if not re.search(r"\bstack|\bpile\b|\btower\b", cmd):
            return None
        if not re.search(r"\b(boxes|mugs|blocks|cubes|objects|items|them)\b", cmd):
            return None
        if re.search(r"\bjenga\b", cmd) and not re.search(r"\bboxes|mugs|blocks", cmd):
            return None
        cls = "box"
        if "mug" in cmd:
            cls = "mug"
        elif re.search(r"\bblock|cube\b", cmd):
            cls = "jenga_block"
        return {
            "goal": f"Stack every {cls} into a single tower",
            "actions": [
                _act("scan_workspace", "Refresh the world model"),
                _act("stack_all", "Iteratively pick and stack matching objects",
                     object_id=cls),
            ],
            "confidence": 0.9,
        }

    def _compile_drop(self, cmd: str, objs: list) -> Optional[dict]:
        if not re.search(r"\bdrop\b|\brelease\b|\blet go\b", cmd):
            return None
        if re.search(r"\bdrop\b.*\b(?:in|into|on)\b.*\b(?:pan|plate|bowl|tray|box|bin|glass|mug|cup)\b", cmd):
            return None  # handled by _compile_put

        ground = bool(re.search(r"\b(ground|floor|down)\b", cmd))
        m = re.search(r"\bdrop\b\s+(?:the\s+)?(.+?)(?:\s+on|\s+to|\s+at|$)", cmd)
        target = _canonical_object(m.group(1)) if m else "object"
        if re.search(r"\b(boxes|objects|items|everything|all)\b", cmd):
            return {
                "goal": "Drop every box on the ground",
                "actions": [
                    _act("scan_workspace", "Refresh the world model"),
                    _act("drop_all", "Pick each matching object and release it on the ground",
                         object_id=target, surface="ground" if ground else "table"),
                ],
                "confidence": 0.9,
            }
        return {
            "goal": f"Pick up {target} and drop it",
            "actions": [
                _act("pick", f"Grasp {target}", object_id=target),
                _act("drop", "Release the held object",
                     surface="ground" if ground else "table"),
            ],
            "confidence": 0.9,
        }

    def _compile_nest_all(self, cmd: str, objs: list) -> Optional[dict]:
        if not re.search(r"\b(put|place|insert)\b", cmd):
            return None
        if not re.search(r"\b(smaller|any|all|each|every)\b", cmd):
            return None
        if not re.search(r"\b(bigger|larger|container|receptacle|objects)\b", cmd):
            return None
        return {
            "goal": "Place every smaller object inside the available larger containers",
            "actions": [
                _act("scan_workspace", "Refresh the world model"),
                _act("nest_all", "Rank objects by volume and nest small into large"),
            ],
            "confidence": 0.88,
        }

    def _compile_stack(self, cmd: str, objs: list) -> Optional[dict]:
        if not re.search(r"\bstack\b", cmd):
            return None
        # "stack X on Y" / "stack X on top of Y"
        m = re.search(r"\bstack\b\s+(.+?)\s+(?:on|onto|on top of|atop)\s+(.+)$", cmd)
        if not m:
            # "stack the jenga tower" → stack a loose block onto the tower
            if "jenga" in cmd:
                return {
                    "goal": "Stack a loose Jenga block onto the Jenga tower",
                    "actions": [
                        _act("scan_workspace", "Refresh the world model"),
                        _act("pick", "Grasp a loose Jenga block",
                             object_id="jenga_block"),
                        _act("stack", "Align over and release onto the tower",
                             base_id="jenga_tower"),
                    ],
                    "confidence": 0.92,
                }
            return None
        obj = _canonical_object(m.group(1))
        base = _canonical_object(m.group(2))
        return {
            "goal": f"Stack {obj} on {base}",
            "actions": [
                _act("scan_workspace", "Refresh the world model"),
                _act("pick", f"Grasp {obj}", object_id=obj),
                _act("stack", f"Align over {base} and release", base_id=base),
                _act("verify", "Confirm the stacked object is stable",
                     object_id=obj),
            ],
            "confidence": 0.93,
        }

    def _compile_put(self, cmd: str, objs: list) -> Optional[dict]:
        split = _split_put(cmd)
        if not split:
            return None
        obj_raw, dest_raw = split
        obj = _canonical_object(obj_raw)
        dest = _canonical_object(dest_raw)
        if not obj or not dest:
            return None
        handle = bool(re.search(r"\bhandle", cmd))
        params: Dict[str, Any] = {}
        if handle:
            params["grasp_strategy"] = "handle"
        return {
            "goal": f"Put {obj} into {dest}",
            "actions": [
                _act("scan_workspace", "Refresh the world model"),
                _act("pick", f"Grasp {obj}", object_id=obj, **params),
                _act("place", f"Carry {obj} to {dest} and release",
                     target=dest, object_id=obj),
                _act("verify", "Confirm the object is contained", object_id=obj),
            ],
            "confidence": 0.93,
        }

    def _compile_pick_handle(self, cmd: str, objs: list) -> Optional[dict]:
        if "handle" not in cmd:
            return None
        if not re.search(r"\bpick|\bgrab|\blift|\btake\b", cmd):
            return None
        m = re.search(
            r"\b(?:pick|grab|take|get|lift)\b(?:\s+up)?\s+(.+?)"
            r"(?:\s+by|\s+using|\s+from|\s+at)", cmd)
        target = _canonical_object(m.group(1)) if m else "mug"
        if target in {"object", ""}:
            target = "mug"
        return {
            "goal": f"Pick up {target} by its handle",
            "actions": [
                _act("scan_workspace", "Refresh the world model"),
                _act("pick", f"Grasp {target} by the handle",
                     object_id=target, grasp_strategy="handle"),
            ],
            "confidence": 0.92,
        }

    def _compile_press(self, cmd: str, objs: list) -> Optional[dict]:
        if not re.search(r"\b(press|push the button|tap|activate the button|e-?stop)\b", cmd):
            return None
        m = re.search(r"\b(?:press|push|tap|activate)\b\s+(.+)$", cmd)
        target = _canonical_object(m.group(1)) if m else "button"
        if target in {"object", ""}:
            target = "button"
        return {
            "goal": f"Press {target}",
            "actions": [
                _act("scan_workspace", "Refresh the world model"),
                _act("press", f"Hover and descend onto {target}", object_id=target),
            ],
            "confidence": 0.9,
        }

    def _compile_find(self, cmd: str, objs: list) -> Optional[dict]:
        if not re.search(r"\b(find|locate|search|where is|look for)\b", cmd):
            return None
        m = re.search(
            r"\b(?:find|locate|search for|look for|where is)\b\s+(.+)$", cmd)
        target = _canonical_object(m.group(1)) if m else "object"
        if not target:
            return None
        return {
            "goal": f"Locate {target}",
            "actions": [
                _act("scan_workspace", "Sweep the workspace"),
                _act("locate", f"Resolve the 3D pose of {target}",
                     object_id=target),
            ],
            "confidence": 0.9,
        }

    def _compile_pick(self, cmd: str, objs: list) -> Optional[dict]:
        if not re.search(r"\b(pick|grab|lift|take|get)\b", cmd):
            return None
        if not re.search(r"\bup\b|\bpick\b|\bgrab\b|\blift\b|\btake\b", cmd):
            return None
        m = re.search(
            r"\b(?:pick|grab|take|get|lift)\b(?:\s+up)?\s+(?:the\s+|a\s+|an\s+)?"
            r"(.+?)(?:\s+(?:and|then|from|by|to)\b|$)", cmd)
        target = _canonical_object(m.group(1)) if m else ""
        if not target or target == "object":
            return None
        return {
            "goal": f"Pick up {target}",
            "actions": [
                _act("scan_workspace", "Refresh the world model"),
                _act("pick", f"Locate, plan the grasp and lift {target}",
                     object_id=target),
            ],
            "confidence": 0.92,
        }

    def _compile_place(self, cmd: str, objs: list) -> Optional[dict]:
        if not re.search(r"\b(place|put|set)\b", cmd):
            return None
        split = _split_put(cmd)
        if not split:
            return None
        obj = _canonical_object(split[0])
        dest = _canonical_object(split[1])
        if not obj or not dest:
            return None
        return {
            "goal": f"Place {obj} at {dest}",
            "actions": [
                _act("scan_workspace", "Refresh the world model"),
                _act("pick", f"Grasp {obj}", object_id=obj),
                _act("place", f"Move to {dest} and release", target=dest),
            ],
            "confidence": 0.9,
        }

    def _compile_scan(self, cmd: str, objs: list) -> Optional[dict]:
        if not re.search(r"\b(scan|look around|survey|refresh)\b", cmd):
            return None
        return {
            "goal": "Scan and rebuild the world model",
            "actions": [_act("scan_workspace", "Sweep the workspace")],
            "confidence": 0.95,
        }


__all__ = ["IntentPlanner"]
