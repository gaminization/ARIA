#!/usr/bin/env python3
"""
Single source of truth for the executable skill vocabulary shared by the
LLM planner (validation / repair prompts) and the SkillAgent (dispatch).
"""

SKILL_ALIAS_MAP = {
    # Core manipulation skills & aliases
    'locate': 'locate', 'locate_object': 'locate', 'locate_target': 'locate',
    'locate_all': 'locate', 'locate_base': 'locate', 'VisionAgent': 'locate',
    'classify': 'inspect', 'classify_all': 'inspect',
    'pick': 'pick', 'pick_each': 'sort', 'pick_all': 'sort',
    'place': 'place', 'place_on': 'place', 'place_in_zone': 'sort',
    'plan_grasp': 'plan_grasp', 'AffordanceAgent': 'plan_grasp',
    'execute_grasp': 'execute_grasp',
    'lift': 'lift',
    'sort': 'sort', 'sort_all': 'sort',
    'transport': 'transport', 'verify': 'verify', 'verify_stable': 'verify',
    'execute_push': 'push', 'push': 'push',
    'execute_pull': 'pull', 'pull': 'pull',
    'align_over': 'stack', 'stack': 'stack',
    'execute_sweep': 'sweep', 'sweep': 'sweep',
    'execute_slide': 'slide', 'slide': 'slide',
    'execute_roll': 'roll', 'roll': 'roll',
    # In-hand manipulation (arm_agents.in_hand_manipulation)
    'rotate_in_hand': 'rotate_in_hand', 'rotate_held': 'rotate_in_hand',
    'reposition_grip': 'reposition_grip', 'regrip': 'reposition_grip',
    'flip': 'flip', 'flip_object': 'flip',
    'slide_to_tip': 'slide_to_tip', 'extend_in_hand': 'slide_to_tip',
    'capture_views': 'inspect', 'inspect': 'inspect',
    # Workspace physical scan (ScanAgent or LLM plan)
    'scan_workspace': 'scan_workspace', 'search_workspace': 'scan_workspace',
    'scan': 'scan_workspace', 'ScanAgent': 'scan_workspace',
    # Perception pipeline agents called by LLM
    'SAM2Agent': 'sam', 'sam2': 'sam', 'sam': 'sam', 'get_sam_mask': 'sam',
    'DepthAgent': 'depth', 'depth': 'depth', 'get_depth': 'depth', 'approach_safe_distance': 'depth',
    'ClearGraspAgent': 'cleargrasp', 'cleargrasp': 'cleargrasp', 'correct_depth': 'cleargrasp',
    'MaterialAgent': 'material', 'material': 'material', 'check_material': 'material',
    'WorldModelAgent': 'world_model', 'world_model': 'world_model', 'MemoryAgent': 'world_model',
    'TrackingAgent': 'track', 'track': 'track',
    'ReachabilityAgent': 'reachability', 'reachability': 'reachability', 'check_reachability': 'reachability',
    'SafetyAgent': 'safety', 'safety': 'safety', 'check_safety': 'safety',
    'GraspNetAgent': 'plan_grasp', 'OpenVLAAgent': 'plan_grasp',
    'SkillAgent': 'pick', 'ControlAgent': 'pick',
    # Free-form motion primitives (LLM authors the motion)
    'joint_motion': 'joint_motion', 'gesture': 'joint_motion', 'wave': 'joint_motion',
    'move_to': 'move_to', 'move_arm': 'move_to', 'move_above': 'move_to', 'reach': 'move_to',
    'press': 'press', 'press_button': 'press', 'tap': 'press', 'type_key': 'press',
    'gripper': 'gripper', 'open_gripper': 'gripper', 'close_gripper': 'gripper', 'release': 'gripper',
    'named_pose': 'named_pose', 'go_home': 'named_pose', 'wait': 'wait',
    # Composite / infrastructure behaviors
    'conveyor': 'conveyor', 'set_conveyor': 'conveyor', 'belt': 'conveyor',
    'conveyor_speed': 'conveyor', 'start_conveyor': 'conveyor',
    'drop': 'drop', 'drop_object': 'drop', 'drop_all': 'drop_all',
    'pick_all': 'pick_all', 'pick_everything': 'pick_all',
    'stack_all': 'stack_all', 'nest_all': 'nest_all',
    'sort_by_color': 'sort_by_color', 'sort_colors': 'sort_by_color',
}


def resolve_executable(skill, agent):
    """Return the executable action name for an LLM plan step, or None if unknown."""
    for cand in (skill, agent):
        if cand and cand in SKILL_ALIAS_MAP:
            return cand
    return None


EXECUTABLE_NAMES = sorted(k for k in SKILL_ALIAS_MAP if k[:1].islower())

# Curated vocabulary shown to the LLM (aliases above are still accepted when resolving).
CORE_SKILLS = {
    'scan_workspace': "sweep the arm so the wrist camera (top-down) re-detects everything; rebuilds the world model",
    'locate': "params: object_id. Find the object in the world model (scans if missing); caches its 3D pose",
    'reachability': "params: object_id. Check the located object is inside the arm's reachable workspace",
    'safety': "check joint feedback and workspace bounds before moving near an object",
    'depth': "params: object_id. Monocular depth (Depth-Anything/MiDaS): object height and safe approach distance",
    'sam': "params: object_id. SAM2 mask to refine the grasp centroid and prevent slipping",
    'material': "params: object_id. Identify material (glass/metal/plastic/fabric)",
    'cleargrasp': "params: object_id. ClearGrasp depth correction for transparent/glass objects",
    'inspect': "params: object_id. Move the wrist camera around the object to capture several views",
    'track': "params: object_id. Measure whether the object is moving (e.g. on a conveyor)",
    'plan_grasp': "params: object_id. AffordanceAgent/grasp planning (handles, special strategies, GraspNet for novel shapes)",
    'execute_grasp': "params: object_id. Approach, visual-servo, descend and close the gripper",
    'lift': "params: object_id. Raise a grasped object clear of the table",
    'pick': "params: object_id. Atomic locate+plan_grasp+execute_grasp+lift",
    'place': "params: object_id, target (receptacle/destination name). Put the held object down there",
    'stack': "params: object_id, base_id. Stack the held object on another",
    'sort': "params: object_ids (list), zone_map. Sort several objects into zones",
    'push': "params: object_id, direction, distance_m", 'pull': "params: object_id, direction, distance_m",
    'verify': "params: object_id. Check from sensors that the outcome really happened (held / placed)",
    'move_to': "params: x,y,z (metres, frame='world'|'base') OR object_id + height_above (m). Move the gripper there via IK",
    'joint_motion': "params: waypoints=[[waist,shoulder,elbow,wrist,gripper] in DEGREES,...], speed_dps, repeat. Free-form gesture/motion",
    'press': "params: object_id (a button/key) or x,y; optional depth_m. Hover over it, descend until contact depth, retract",
    'gripper': "params: state='open'|'close' or angle_deg (0 closed .. 55 open)",
    'named_pose': "params: pose='home'|'ready'|...",
    'wait': "params: seconds",
}

