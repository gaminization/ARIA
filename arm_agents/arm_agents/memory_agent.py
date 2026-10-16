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
        self.track_to_object: Dict[int, int] = {}

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
        self.bus.on_change('memory', self._on_external_memory)
        self.create_service(Trigger, '/aria/reset_memory', self._reset_memory_cb)
        self.create_timer(0.5, self._publish_state)  # 2Hz
        return TransitionCallbackReturn.SUCCESS

    def _reset_memory_cb(self, req, res):
        """Service callback to completely wipe active objects and DB on user reset."""
        self.objects.clear()
        self.unconfirmed_candidates.clear()
        self.track_to_object.clear()
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

    def _on_external_memory(self, msg: MemoryState):
        """Retain lifecycle changes published by agents that perform manipulation."""
        for incoming in msg.known_objects:
            if incoming.lifecycle_state not in {
                'Detected', 'Tracked', 'Held', 'Grasped', 'Moved', 'Released'
            }:
                continue
            next_state = 'Tracked' if incoming.lifecycle_state == 'Released' else incoming.lifecycle_state
            existing = self.objects.get(incoming.id)
            if (existing is None or existing.name != incoming.name or
                    existing.class_name != incoming.class_name):
                named_matches = [
                    candidate for candidate in self.objects.values()
                    if candidate.name == incoming.name
                    and candidate.class_name == incoming.class_name
                ]
                if len(named_matches) == 1:
                    existing = named_matches[0]
                else:
                    existing = None

            if existing is None:
                incoming_pos = incoming.last_known_pose.pose.position
                candidates = []
                for candidate in self.objects.values():
                    if candidate.class_name != incoming.class_name:
                        continue
                    if incoming.lifecycle_state not in {
                        'Detected', 'Tracked', 'Held', 'Grasped'
                    } and not (
                        incoming.lifecycle_state == 'Released'
                        and candidate.lifecycle_state in {'Held', 'Grasped'}
                    ):
                        continue
                    if incoming.color and candidate.color and incoming.color != candidate.color:
                        continue
                    candidate_pos = candidate.last_known_pose.pose.position
                    distance = math.hypot(
                        candidate_pos.x - incoming_pos.x,
                        candidate_pos.y - incoming_pos.y,
                    )
                    if distance <= 0.03:
                        candidates.append((distance, candidate))
                candidates.sort(key=lambda item: item[0])
                if candidates and not (
                    len(candidates) > 1 and candidates[1][0] - candidates[0][0] < 0.005
                ):
                    existing = candidates[0][1]

            if existing is None and incoming.lifecycle_state in {
                'Detected', 'Tracked', 'Held', 'Grasped'
            }:
                existing = WorldObject()
                existing.id = self.next_id
                self.next_id += 1
                existing.name = incoming.name
                existing.class_name = incoming.class_name
                existing.last_known_pose = incoming.last_known_pose
                existing.color = incoming.color
                existing.material = incoming.material
                existing.lifecycle_state = incoming.lifecycle_state
                existing.last_seen = incoming.last_seen
                if not existing.last_seen.sec and not existing.last_seen.nanosec:
                    existing.last_seen.sec = int(time.time())
                self.objects[existing.id] = existing

            if existing is None:
                continue
            if not (
                existing.lifecycle_state in {'Held', 'Grasped'}
                and incoming.lifecycle_state in {'Detected', 'Tracked'}
            ):
                existing.lifecycle_state = next_state
            self._save_object(existing)

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

    # Workspace gating bounds (optical table, conveyors, and elevated side pedestals)
    TABLE_X_MIN, TABLE_X_MAX = -0.45, 0.45
    TABLE_Y_MIN, TABLE_Y_MAX = -0.45, 0.85
    TABLE_Z_MIN, TABLE_Z_MAX = 0.50, 0.95
    ARM_BASE_RADIUS = 0.075

    def _on_vision(self, msg: VisionState):
        """
        Update the world model while preserving the identity of nearby, same-class objects.
        """
        now = time.time()
        now_sec = int(now)
        now_nsec = int((now - now_sec) * 1e9)

        association_radius = 0.03
        generic_classes = {'table_object', 'unknown', 'yellow_items', 'white_dish', 'item'}
        detections = []
        for det in msg.detected_objects:
            is_duplicate = False
            for previous in detections:
                ax1 = det.bbox_x - det.bbox_w / 2
                ay1 = det.bbox_y - det.bbox_h / 2
                ax2 = det.bbox_x + det.bbox_w / 2
                ay2 = det.bbox_y + det.bbox_h / 2
                bx1 = previous.bbox_x - previous.bbox_w / 2
                by1 = previous.bbox_y - previous.bbox_h / 2
                bx2 = previous.bbox_x + previous.bbox_w / 2
                by2 = previous.bbox_y + previous.bbox_h / 2
                intersection = max(0.0, min(ax2, bx2) - max(ax1, bx1)) * max(
                    0.0, min(ay2, by2) - max(ay1, by1)
                )
                area_a = det.bbox_w * det.bbox_h
                area_b = previous.bbox_w * previous.bbox_h
                iou = intersection / (area_a + area_b - intersection + 1e-6)
                pos_a = det.pose_3d.pose.position
                pos_b = previous.pose_3d.pose.position
                same_pose = math.hypot(pos_a.x - pos_b.x, pos_a.y - pos_b.y) <= 0.025
                if iou >= 0.85 and same_pose:
                    is_duplicate = True
                    break
            if not is_duplicate:
                detections.append(det)

        # Collect current frame detection locations by class
        current_frame_dets_by_class = {}
        for det in detections:
            c = det.class_name.lower().strip()
            if c:
                current_frame_dets_by_class.setdefault(c, []).append((det.pose_3d.pose.position.x, det.pose_3d.pose.position.y))

        seen_track_ids = set()
        matched_object_ids = set()
        matched_candidate_ids = set()
        for det in detections:
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

            matched_wo = None
            track_id = int(det.tracking_id)
            if track_id in seen_track_ids:
                track_id = -1
            elif track_id >= 0:
                seen_track_ids.add(track_id)

            # Tracking IDs preserve identity even when nearby instances share a class.
            if track_id >= 0:
                mapped_id = self.track_to_object.get(track_id)
                if mapped_id in self.objects and mapped_id not in matched_object_ids:
                    matched_wo = self.objects[mapped_id]

            if matched_wo is None:
                compatible_objects = []
                for wo in self.objects.values():
                    if wo.id in matched_object_ids:
                        continue
                    same_class = wo.class_name == det.class_name
                    generic_match = wo.class_name in generic_classes or det.class_name in generic_classes
                    color_matches = not det_color or not wo.color or det_color == wo.color
                    if not (same_class or generic_match) or not color_matches:
                        continue
                    wo_pos = wo.last_known_pose.pose.position
                    distance = math.hypot(wo_pos.x - det_x, wo_pos.y - det_y)
                    if distance <= association_radius:
                        compatible_objects.append((distance, wo))
                if compatible_objects:
                    matched_wo = min(compatible_objects, key=lambda item: item[0])[1]

            if matched_wo is None:
                # A changed location indicates relocation only when its old position is empty.
                relocation_candidates = []
                for wo in self.objects.values():
                    if wo.id in matched_object_ids or wo.class_name != det.class_name:
                        continue
                    if det_color and wo.color and det_color != wo.color:
                        continue
                    old_pos = wo.last_known_pose.pose.position
                    current_detections = current_frame_dets_by_class.get(det.class_name.lower().strip(), [])
                    still_at_old_location = any(
                        math.hypot(dx - old_pos.x, dy - old_pos.y) <= association_radius
                        for dx, dy in current_detections
                    )
                    if not still_at_old_location or wo.lifecycle_state in {'Held', 'Moved'}:
                        relocation_candidates.append(
                            (math.hypot(old_pos.x - det_x, old_pos.y - det_y), wo)
                        )
                if relocation_candidates:
                    _, matched_wo = min(relocation_candidates, key=lambda item: item[0])
                    matched_wo.lifecycle_state = 'Moved'

            if matched_wo is not None:
                old_pos = matched_wo.last_known_pose.pose.position
                displacement = math.hypot(old_pos.x - det_x, old_pos.y - det_y)
                prior_state = matched_wo.lifecycle_state
                alpha = 1.0 if prior_state in {'Moved', 'Held', 'Grasped'} else (
                    0.75 if displacement > 0.01 else 0.25
                )
                matched_wo.last_known_pose.pose.position.x = (1.0 - alpha) * matched_wo.last_known_pose.pose.position.x + alpha * det_x
                matched_wo.last_known_pose.pose.position.y = (1.0 - alpha) * matched_wo.last_known_pose.pose.position.y + alpha * det_y
                matched_wo.last_known_pose.pose.position.z = (1.0 - alpha) * matched_wo.last_known_pose.pose.position.z + alpha * det_z
                matched_wo.last_seen.sec = now_sec
                matched_wo.last_seen.nanosec = now_nsec
                matched_wo.lifecycle_state = (
                    prior_state if prior_state in {'Held', 'Grasped'} else 'Tracked'
                )

                # Upgrade generic or missing color
                if det_color and not matched_wo.color:
                    matched_wo.color = det_color
                if matched_wo.class_name in generic_classes and det.class_name not in generic_classes:
                    matched_wo.class_name = det.class_name
                    matched_wo.name = f"{det.class_name}_{matched_wo.id}"

                matched_object_ids.add(matched_wo.id)
                if track_id >= 0:
                    self.track_to_object[track_id] = matched_wo.id
                self._save_object(matched_wo)
            else:
                # Temporal Confirmation Filter: require 3 frame sightings within 2.0s
                self.unconfirmed_candidates = {
                    cid: c for cid, c in self.unconfirmed_candidates.items()
                    if now - c['last_seen'] < 2.0
                }

                # Prefer a stable track ID; otherwise use a tight spatial match.
                best_cand_id = None
                cand_min_dist = float('inf')
                for cid, c in self.unconfirmed_candidates.items():
                    if cid in matched_candidate_ids:
                        continue
                    is_same_or_generic = (
                        c['class_name'] == det.class_name or
                        c['class_name'] in generic_classes or
                        det.class_name in generic_classes
                    )
                    if not is_same_or_generic:
                        continue
                    d = math.hypot(c['x'] - det_x, c['y'] - det_y)
                    same_track = track_id >= 0 and c.get('tracking_id') == track_id
                    if (same_track or d <= association_radius) and d < cand_min_dist:
                        cand_min_dist = d
                        best_cand_id = cid

                if best_cand_id is not None:
                    matched_candidate_ids.add(best_cand_id)
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
                        existing_same = None
                        c_cls = c['class_name']
                        c_col = c.get('color', det_color)
                        for o in self.objects.values():
                            if o.id in matched_object_ids or o.class_name != c_cls:
                                continue
                            if c_col and o.color and c_col != o.color:
                                continue
                            o_pos = o.last_known_pose.pose.position
                            if math.hypot(o_pos.x - c['x'], o_pos.y - c['y']) <= association_radius:
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
                            matched_object_ids.add(existing_same.id)
                            if track_id >= 0:
                                self.track_to_object[track_id] = existing_same.id
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
                        matched_object_ids.add(wo.id)
                        if track_id >= 0:
                            self.track_to_object[track_id] = wo.id
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
                        'class_name': det.class_name,
                        'tracking_id': track_id if track_id >= 0 else None,
                    }

    def _publish_state(self):
        now = time.time()
        # Stale object pruning: retire objects unseen for > 5.0 seconds (or Held objects unseen for > 60s)
        stale_ids = []
        for oid, wo in self.objects.items():
            last_seen_s = float(wo.last_seen.sec) + float(wo.last_seen.nanosec) * 1e-9
            max_unseen_s = 60.0 if wo.lifecycle_state in {'Held', 'Grasped'} else 5.0
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
        if stale_ids:
            stale_id_set = set(stale_ids)
            self.track_to_object = {
                track_id: object_id
                for track_id, object_id in self.track_to_object.items()
                if object_id not in stale_id_set
            }

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
