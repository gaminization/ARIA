#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Memory Agent — LifecycleNode
Persistent object storage. SQLite-backed WorldModel.
Publishes MemoryState to state bus at 2Hz.
═══════════════════════════════════════════════════════════════
"""
import os, time, sqlite3, math
from typing import Dict, Optional
import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from arm_planner.msg import MemoryState, WorldObject, SemanticRelation, VisionState
from arm_planner.state_bus import StateBus

class MemoryAgent(LifecycleNode):
    """
    Persistent object memory manager.

    On detection:
      New → create WorldObject, assign ID
      Known → update last_known_pose, last_seen

    When object not at expected location:
      → DialogueAgent asks user

    SQLite persistence at arm_planner/data/world_model.db
    """

    def __init__(self):
        super().__init__('memory_agent')
        self.bus = StateBus(self)
        self.objects: Dict[int, WorldObject] = {}
        self.relations: list = []
        self.recent_tasks: list = []
        self.next_id = 1
        self.unconfirmed_candidates: dict = {}
        self.next_cand_id = 1

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("MemoryAgent: CONFIGURING")
        self.declare_parameter('db_path', '')
        self.declare_parameter('clear_stale_on_start', True)
        db_path = self.get_parameter('db_path').value
        if not db_path:
            db_path = os.path.join(
                os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                '..', 'arm_planner', 'data', 'world_model.db')
        self.db_path = db_path
        self._init_db()
        if self.get_parameter('clear_stale_on_start').value:
            try:
                conn = sqlite3.connect(self.db_path)
                conn.execute('DELETE FROM objects')
                conn.commit()
                conn.close()
                self.objects.clear()
                self.next_id = 1
                self.get_logger().info("MemoryAgent: Initialized clean session world model database")
            except Exception as e:
                self.get_logger().warn(f"Failed to clear stale DB: {e}")
        else:
            self._load_from_db()
        self.bus.on_change('vision', self._on_vision)
        self.create_service(Trigger, '/aria/reset_memory', self._reset_memory_cb)
        self.create_timer(0.5, self._publish_state)  # 2Hz
        return TransitionCallbackReturn.SUCCESS

    def _reset_memory_cb(self, req, res):
        """Service callback to completely wipe active objects and DB on user reset."""
        self.objects.clear()
        self.unconfirmed_candidates.clear()
        self.next_id = 1
        try:
            conn = sqlite3.connect(self.db_path)
            conn.execute('DELETE FROM objects')
            conn.commit()
            conn.close()
        except Exception:
            pass
        self._publish_state()
        res.success = True
        res.message = "Memory agent wiped and republished"
        return res

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"MemoryAgent: ACTIVATED — {len(self.objects)} objects in memory")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._save_all()
        return TransitionCallbackReturn.SUCCESS

    def _init_db(self):
        os.makedirs(os.path.dirname(self.db_path), exist_ok=True)
        conn = sqlite3.connect(self.db_path)
        conn.execute('''CREATE TABLE IF NOT EXISTS objects (
            id INTEGER PRIMARY KEY,
            name TEXT, class_name TEXT,
            px REAL, py REAL, pz REAL,
            last_seen REAL,
            color TEXT, material TEXT,
            lifecycle_state TEXT
        )''')
        conn.execute('''CREATE TABLE IF NOT EXISTS relations (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            subject TEXT, relation TEXT, object TEXT,
            confidence REAL, distance REAL
        )''')
        conn.commit()
        conn.close()

    def _load_from_db(self):
        conn = sqlite3.connect(self.db_path)
        for row in conn.execute('SELECT * FROM objects'):
            wo = WorldObject()
            wo.id = row[0]
            wo.name = row[1] or ''
            wo.class_name = row[2] or ''
            wo.last_known_pose = PoseStamped()
            wo.last_known_pose.header.frame_id = 'world'
            wo.last_known_pose.pose.position.x = row[3] or 0.0
            wo.last_known_pose.pose.position.y = row[4] or 0.0
            wo.last_known_pose.pose.position.z = row[5] or 0.0
            wo.color = row[7] or ''
            wo.material = row[8] or ''
            wo.lifecycle_state = row[9] or 'Known'
            self.objects[wo.id] = wo
            self.next_id = max(self.next_id, wo.id + 1)
        conn.close()

    def _save_object(self, wo: WorldObject):
        try:
            self._init_db()
            conn = sqlite3.connect(self.db_path)
            conn.execute('''INSERT OR REPLACE INTO objects
                (id, name, class_name, px, py, pz, last_seen, color, material, lifecycle_state)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)''', (
                wo.id, wo.name, wo.class_name,
                wo.last_known_pose.pose.position.x,
                wo.last_known_pose.pose.position.y,
                wo.last_known_pose.pose.position.z,
                time.time(), wo.color, wo.material, wo.lifecycle_state))
            conn.commit()
            conn.close()
        except Exception:
            pass

    def _save_all(self):
        for wo in self.objects.values():
            self._save_object(wo)

    # Workspace gating bounds (optical table and elevated side pedestals)
    TABLE_X_MIN, TABLE_X_MAX = -0.32, 0.35
    TABLE_Y_MIN, TABLE_Y_MAX = -0.35, 0.35
    TABLE_Z_MIN, TABLE_Z_MAX = 0.55, 0.85
    ARM_BASE_RADIUS = 0.075

    def _on_vision(self, msg: VisionState):
        """
        Update memory from vision detections with industrial multi-instance spatial association.
        Supports multiple distinct objects of the same class (e.g. mug_1, mug_2, block_1, block_2)
        while tracking relocated/moved workpieces and preventing duplicate instance proliferation.
        """
        now = time.time()
        now_sec = int(now)
        now_nsec = int((now - now_sec) * 1e9)

        # Collect current frame detection locations by class
        current_frame_dets_by_class = {}
        for det in msg.detected_objects:
            c = det.class_name.lower().strip()
            if c:
                current_frame_dets_by_class.setdefault(c, []).append((det.pose_3d.pose.position.x, det.pose_3d.pose.position.y))

        for det in msg.detected_objects:
            if not det.class_name or det.class_name in {'unknown', 'dining table', 'chair'}:
                continue

            det_x = det.pose_3d.pose.position.x
            det_y = det.pose_3d.pose.position.y
            det_z = det.pose_3d.pose.position.z

            # Extract color from affordance_regions if present
            det_color = ''
            for aff in det.affordance_regions:
                if aff.startswith('color:'):
                    det_color = aff.split('color:', 1)[1].lower().strip()
                    break

            # 1. Outlier & workspace boundary gating
            if not (self.TABLE_X_MIN <= det_x <= self.TABLE_X_MAX and
                    self.TABLE_Y_MIN <= det_y <= self.TABLE_Y_MAX and
                    self.TABLE_Z_MIN <= det_z <= self.TABLE_Z_MAX):
                continue

            # Exclude robot arm mounting base center
            if math.hypot(det_x, det_y) < self.ARM_BASE_RADIUS:
                continue

            # 2. Spatial proximity association:
            closest_wo = None
            min_dist = float('inf')
            same_class_closest = None
            same_class_min_dist = float('inf')

            for wo in self.objects.values():
                wo_x = wo.last_known_pose.pose.position.x
                wo_y = wo.last_known_pose.pose.position.y
                d = math.hypot(wo_x - det_x, wo_y - det_y)
                if d < min_dist:
                    min_dist = d
                    closest_wo = wo
                if wo.class_name == det.class_name:
                    # Match same class; prioritize color match if both have color
                    if not det_color or not wo.color or det_color == wo.color:
                        if d < same_class_min_dist:
                            same_class_min_dist = d
                            same_class_closest = wo

            # Check if this matches an existing workpiece:
            matched_wo = None
            generic_classes = {'table_object', 'unknown', 'yellow_items', 'white_dish', 'item'}
            if same_class_closest is not None and same_class_min_dist < 0.075:
                matched_wo = same_class_closest
            elif closest_wo is not None and (closest_wo.class_name in generic_classes or det.class_name in generic_classes) and min_dist < 0.060:
                matched_wo = closest_wo
            elif closest_wo is not None and closest_wo.class_name == det.class_name and min_dist < 0.050:
                matched_wo = closest_wo
            else:
                # Dynamic Relocation Check:
                # If an existing object of same class (and matching color) exists,
                # check if there is NO detection near its old location in this frame,
                # or if the object was held/moved by the robot arm.
                for wo in self.objects.values():
                    if wo.class_name == det.class_name and (not det_color or not wo.color or det_color == wo.color):
                        old_x = wo.last_known_pose.pose.position.x
                        old_y = wo.last_known_pose.pose.position.y
                        # Check if any detection in current frame is near old location
                        dets_for_cls = current_frame_dets_by_class.get(det.class_name.lower().strip(), [])
                        has_det_at_old = any(math.hypot(dx - old_x, dy - old_y) < 0.075 for dx, dy in dets_for_cls)
                        if not has_det_at_old or wo.lifecycle_state in {'Held', 'Moved'}:
                            # Old location is empty — workpiece was moved!
                            matched_wo = wo
                            wo.lifecycle_state = 'Moved'
                            break

            if matched_wo is not None:
                # Observation of existing workpiece: smooth update position (EMA alpha=0.25)
                alpha = 0.25 if matched_wo.lifecycle_state != 'Moved' else 1.0
                matched_wo.last_known_pose.pose.position.x = (1.0 - alpha) * matched_wo.last_known_pose.pose.position.x + alpha * det_x
                matched_wo.last_known_pose.pose.position.y = (1.0 - alpha) * matched_wo.last_known_pose.pose.position.y + alpha * det_y
                matched_wo.last_known_pose.pose.position.z = (1.0 - alpha) * matched_wo.last_known_pose.pose.position.z + alpha * det_z
                matched_wo.last_seen.sec = now_sec
                matched_wo.last_seen.nanosec = now_nsec
                matched_wo.lifecycle_state = 'Tracked'

                # Upgrade generic or missing color
                if det_color and not matched_wo.color:
                    matched_wo.color = det_color
                if matched_wo.class_name in generic_classes and det.class_name not in generic_classes:
                    matched_wo.class_name = det.class_name
                    matched_wo.name = f"{det.class_name}_{matched_wo.id}"

                self._save_object(matched_wo)
            else:
                # Temporal Confirmation Filter: require 3 frame sightings within 2.0s
                self.unconfirmed_candidates = {
                    cid: c for cid, c in self.unconfirmed_candidates.items()
                    if now - c['last_seen'] < 2.0
                }

                # Find existing matching candidate within 5cm with same or generic class
                best_cand_id = None
                cand_min_dist = float('inf')
                for cid, c in self.unconfirmed_candidates.items():
                    is_same_or_generic = (
                        c['class_name'] == det.class_name or
                        c['class_name'] in {'unknown', 'table_object'} or
                        det.class_name in {'unknown', 'table_object'}
                    )
                    if not is_same_or_generic:
                        continue
                    d = math.hypot(c['x'] - det_x, c['y'] - det_y)
                    if d < 0.055 and d < cand_min_dist:
                        cand_min_dist = d
                        best_cand_id = cid

                if best_cand_id is not None:
                    c = self.unconfirmed_candidates[best_cand_id]
                    c['count'] += 1
                    c['last_seen'] = now
                    c['x'] = 0.8 * c['x'] + 0.2 * det_x
                    c['y'] = 0.8 * c['y'] + 0.2 * det_y
                    c['z'] = 0.8 * c['z'] + 0.2 * det_z
                    if det_color:
                        c['color'] = det_color
                    if det.class_name not in {'unknown', 'table_object'}:
                        c['class_name'] = det.class_name

                    # Promote candidate to WorldObject after 3 confirmed frames
                    if c['count'] >= 3:
                        # Check if a singleton object of this class and color already exists in self.objects
                        existing_same = None
                        c_cls = c['class_name']
                        c_col = c.get('color', det_color)
                        for o in self.objects.values():
                            if o.class_name == c_cls:
                                if not c_col or not o.color or c_col == o.color:
                                    if c_cls not in {'jenga_block', 'jenga_tower'}:
                                        existing_same = o
                                        break

                        if existing_same is not None:
                            existing_same.last_known_pose.pose.position.x = c['x']
                            existing_same.last_known_pose.pose.position.y = c['y']
                            existing_same.last_known_pose.pose.position.z = c['z']
                            existing_same.last_seen.sec = now_sec
                            existing_same.last_seen.nanosec = now_nsec
                            existing_same.lifecycle_state = 'Tracked'
                            if c_col and not existing_same.color:
                                existing_same.color = c_col
                            self._save_object(existing_same)
                            del self.unconfirmed_candidates[best_cand_id]
                            continue

                        wo = WorldObject()
                        wo.id = self.next_id
                        self.next_id += 1
                        class_count = sum(1 for o in self.objects.values() if o.class_name == c['class_name'])
                        wo.name = f"{c['class_name']}_{class_count + 1}"
                        wo.class_name = c['class_name']
                        wo.last_known_pose = det.pose_3d
                        wo.last_known_pose.pose.position.x = c['x']
                        wo.last_known_pose.pose.position.y = c['y']
                        wo.last_known_pose.pose.position.z = c['z']
                        wo.last_seen.sec = now_sec
                        wo.last_seen.nanosec = now_nsec
                        wo.lifecycle_state = 'Detected'
                        wo.color = c.get('color', det_color)
                        wo.material = ''
                        self.objects[wo.id] = wo
                        self._save_object(wo)
                        del self.unconfirmed_candidates[best_cand_id]
                        self.bus.add_chain_of_thought(
                            f"MEMORY: Confirmed new workpiece: {wo.name} (color='{wo.color}') at (X={c['x']:.3f}, Y={c['y']:.3f}, Z={c['z']:.3f})")
                else:
                    cid = self.next_cand_id
                    self.next_cand_id += 1
                    self.unconfirmed_candidates[cid] = {
                        'count': 1,
                        'last_seen': now,
                        'x': det_x,
                        'y': det_y,
                        'z': det_z,
                        'color': det_color,
                        'class_name': det.class_name
                    }

    def _publish_state(self):
        now = time.time()
        # Stale object pruning: retire objects unseen for > 5.0 seconds (or Held objects unseen for > 60s)
        stale_ids = []
        for oid, wo in self.objects.items():
            last_seen_s = float(wo.last_seen.sec) + float(wo.last_seen.nanosec) * 1e-9
            max_unseen_s = 60.0 if wo.lifecycle_state == 'Held' else 5.0
            if last_seen_s > 0 and (now - last_seen_s) > max_unseen_s:
                stale_ids.append(oid)
        for oid in stale_ids:
            try:
                conn = sqlite3.connect(self.db_path)
                conn.execute('DELETE FROM objects WHERE id = ?', (oid,))
                conn.commit()
                conn.close()
            except Exception:
                pass
            del self.objects[oid]

        msg = MemoryState()
        msg.known_objects = list(self.objects.values())
        msg.spatial_relations = list(self.relations)
        msg.recent_tasks = list(self.recent_tasks[-10:])
        self.bus.publish_memory(msg)

    def add_task_to_history(self, task_description: str):
        self.recent_tasks.append(task_description)
        if len(self.recent_tasks) > 100:
            self.recent_tasks = self.recent_tasks[-50:]

def main(args=None):
    rclpy.init(args=args)
    node = MemoryAgent()
    node.trigger_configure()
    node.trigger_activate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
