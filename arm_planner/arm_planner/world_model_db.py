#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA World Model DB — SQLite-Backed Persistent World Model
Persists between sessions. Full lifecycle management.
Spatial relation computation. YAML import/export.
═══════════════════════════════════════════════════════════════
"""
import json
import math
import os
import sqlite3
import time
from datetime import datetime, timezone
from typing import Dict, List, Optional, Tuple

import yaml

from arm_planner.msg import ObjectDetection, WorldObject, SemanticRelation


# ═══════════════════════════════════════════════════════════════
# Lifecycle States
# ═══════════════════════════════════════════════════════════════
LIFECYCLE_NONE = "NONE"
LIFECYCLE_DETECTED = "DETECTED"
LIFECYCLE_TRACKED = "TRACKED"
LIFECYCLE_LOST = "LOST"
LIFECYCLE_RECOVERED = "RECOVERED"
LIFECYCLE_MOVED = "MOVED"
LIFECYCLE_REMOVED = "REMOVED"

# Transition thresholds
DETECT_CONFIDENCE_THRESHOLD = 0.7
TRACKED_CONSECUTIVE_FRAMES = 3
LOST_FRAMES = 30          # ~1s at 30fps
RECOVERED_TIMEOUT_S = 60.0
MOVED_DISTANCE_M = 0.05   # 5cm
REMOVED_FRAMES = 300       # ~10s at 30fps

# Spatial relation thresholds
NEAR_THRESHOLD_M = 0.10
ON_VERTICAL_M = 0.05
ON_HORIZONTAL_M = 0.03
INSIDE_THRESHOLD_M = 0.02

# Table dimensions (from SDF)
TABLE_CENTER = (0.30, 0.0, 0.76)
TABLE_HALF_SIZE = (0.30, 0.30, 0.02)


class WorldModelDB:
    """
    SQLite-backed persistent world model.

    Features:
      - Object lifecycle management (DETECTED→TRACKED→LOST→...)
      - Automatic spatial relation computation
      - Task history logging
      - YAML import/export for debugging
      - Thread-safe (SQLite handles concurrency)
    """

    def __init__(self, db_path: str = ""):
        if not db_path:
            db_path = os.path.join(
                os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                "data", "world_model.db")
        self.db_path = db_path
        os.makedirs(os.path.dirname(self.db_path), exist_ok=True)
        self._init_db()

        # In-memory tracking for lifecycle transitions
        self._consecutive_frames: Dict[int, int] = {}  # obj_id → consecutive detection count
        self._frames_since_seen: Dict[int, int] = {}   # obj_id → frames since last seen
        self._last_positions: Dict[int, Tuple[float, float, float]] = {}

    # ═══════════════════════════════════════════════════════
    # Database Initialization
    # ═══════════════════════════════════════════════════════
    def _init_db(self):
        """Create tables if they don't exist."""
        conn = sqlite3.connect(self.db_path)
        conn.executescript("""
            CREATE TABLE IF NOT EXISTS objects (
                id INTEGER PRIMARY KEY,
                class_name TEXT NOT NULL,
                display_name TEXT,
                pos_x REAL DEFAULT 0.0,
                pos_y REAL DEFAULT 0.0,
                pos_z REAL DEFAULT 0.0,
                rot_x REAL DEFAULT 0.0,
                rot_y REAL DEFAULT 0.0,
                rot_z REAL DEFAULT 0.0,
                rot_w REAL DEFAULT 1.0,
                color TEXT DEFAULT '',
                material TEXT DEFAULT '',
                size_x REAL DEFAULT 0.04,
                size_y REAL DEFAULT 0.04,
                size_z REAL DEFAULT 0.04,
                lifecycle_state TEXT DEFAULT 'NONE',
                last_seen_ts TEXT,
                detection_count INTEGER DEFAULT 0,
                affordance_scores TEXT DEFAULT '{}'
            );

            CREATE TABLE IF NOT EXISTS spatial_relations (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                subject_id INTEGER NOT NULL,
                relation_type TEXT NOT NULL,
                object_id INTEGER NOT NULL,
                distance_m REAL DEFAULT 0.0,
                confidence REAL DEFAULT 0.0,
                last_updated TEXT,
                FOREIGN KEY (subject_id) REFERENCES objects(id),
                FOREIGN KEY (object_id) REFERENCES objects(id),
                UNIQUE(subject_id, relation_type, object_id)
            );

            CREATE TABLE IF NOT EXISTS task_history (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                command TEXT,
                success INTEGER DEFAULT 0,
                duration_s REAL DEFAULT 0.0,
                failure_reason TEXT DEFAULT '',
                timestamp TEXT
            );

            CREATE INDEX IF NOT EXISTS idx_objects_class
                ON objects(class_name);
            CREATE INDEX IF NOT EXISTS idx_objects_lifecycle
                ON objects(lifecycle_state);
            CREATE INDEX IF NOT EXISTS idx_relations_subject
                ON spatial_relations(subject_id);
            CREATE INDEX IF NOT EXISTS idx_relations_object
                ON spatial_relations(object_id);
        """)
        conn.commit()
        conn.close()

    def _conn(self) -> sqlite3.Connection:
        """Get a new connection (thread-safe pattern)."""
        conn = sqlite3.connect(self.db_path)
        conn.row_factory = sqlite3.Row
        return conn

    def _now_iso(self) -> str:
        return datetime.now(timezone.utc).isoformat()

    # ═══════════════════════════════════════════════════════
    # Object CRUD
    # ═══════════════════════════════════════════════════════
    def upsert_object(self, detection: ObjectDetection) -> int:
        """
        Insert or update an object from a detection.

        Lifecycle transitions:
          NONE → DETECTED: first detection with confidence > 0.7
          DETECTED → TRACKED: detected in 3+ consecutive frames
          LOST → RECOVERED: re-detected within 60s

        Returns: object_id
        """
        conn = self._conn()

        # Try to find existing object by class and proximity
        existing = self._find_matching_object(
            conn, detection.class_name,
            detection.pose_3d.pose.position.x,
            detection.pose_3d.pose.position.y,
            detection.pose_3d.pose.position.z,
        )

        now = self._now_iso()

        if existing is not None:
            obj_id = existing["id"]
            old_state = existing["lifecycle_state"]
            old_pos = (existing["pos_x"], existing["pos_y"], existing["pos_z"])
            new_pos = (
                detection.pose_3d.pose.position.x,
                detection.pose_3d.pose.position.y,
                detection.pose_3d.pose.position.z,
            )

            # Compute position change
            dist = math.sqrt(sum((a - b) ** 2 for a, b in zip(old_pos, new_pos)))

            # Lifecycle transitions
            new_state = old_state
            if old_state == LIFECYCLE_LOST:
                new_state = LIFECYCLE_RECOVERED
            elif old_state in (LIFECYCLE_DETECTED, LIFECYCLE_TRACKED, LIFECYCLE_RECOVERED):
                # Track consecutive frames
                self._consecutive_frames[obj_id] = \
                    self._consecutive_frames.get(obj_id, 0) + 1
                if old_state == LIFECYCLE_DETECTED and \
                        self._consecutive_frames.get(obj_id, 0) >= TRACKED_CONSECUTIVE_FRAMES:
                    new_state = LIFECYCLE_TRACKED
                elif dist > MOVED_DISTANCE_M:
                    new_state = LIFECYCLE_MOVED
                else:
                    new_state = LIFECYCLE_TRACKED

            # Reset unseen counter
            self._frames_since_seen[obj_id] = 0

            # Update
            conn.execute("""
                UPDATE objects SET
                    pos_x=?, pos_y=?, pos_z=?,
                    rot_x=?, rot_y=?, rot_z=?, rot_w=?,
                    lifecycle_state=?,
                    last_seen_ts=?,
                    detection_count = detection_count + 1
                WHERE id=?
            """, (
                new_pos[0], new_pos[1], new_pos[2],
                detection.pose_3d.pose.orientation.x,
                detection.pose_3d.pose.orientation.y,
                detection.pose_3d.pose.orientation.z,
                detection.pose_3d.pose.orientation.w,
                new_state, now, obj_id,
            ))
            conn.commit()
            conn.close()
            return obj_id

        else:
            # New object
            if detection.confidence < DETECT_CONFIDENCE_THRESHOLD:
                conn.close()
                return -1  # Below threshold, don't register

            cursor = conn.execute("""
                INSERT INTO objects
                    (class_name, display_name,
                     pos_x, pos_y, pos_z,
                     rot_x, rot_y, rot_z, rot_w,
                     lifecycle_state, last_seen_ts, detection_count,
                     affordance_scores)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, 1, '{}')
            """, (
                detection.class_name,
                f"{detection.class_name}",
                detection.pose_3d.pose.position.x,
                detection.pose_3d.pose.position.y,
                detection.pose_3d.pose.position.z,
                detection.pose_3d.pose.orientation.x,
                detection.pose_3d.pose.orientation.y,
                detection.pose_3d.pose.orientation.z,
                detection.pose_3d.pose.orientation.w,
                LIFECYCLE_DETECTED, now,
            ))
            obj_id = cursor.lastrowid
            conn.commit()
            conn.close()

            self._consecutive_frames[obj_id] = 1
            self._frames_since_seen[obj_id] = 0
            return obj_id

    def _find_matching_object(self, conn, class_name: str,
                              x: float, y: float, z: float,
                              max_dist: float = 0.10) -> Optional[dict]:
        """Find existing object by class and proximity."""
        rows = conn.execute(
            "SELECT * FROM objects WHERE class_name = ? "
            "AND lifecycle_state NOT IN ('REMOVED')",
            (class_name,)
        ).fetchall()

        best = None
        best_dist = max_dist
        for row in rows:
            d = math.sqrt(
                (row["pos_x"] - x) ** 2 +
                (row["pos_y"] - y) ** 2 +
                (row["pos_z"] - z) ** 2
            )
            if d < best_dist:
                best_dist = d
                best = dict(row)
        return best

    def get_object(self, obj_id: int) -> Optional[dict]:
        """Get object by ID."""
        conn = self._conn()
        row = conn.execute(
            "SELECT * FROM objects WHERE id=?", (obj_id,)
        ).fetchone()
        conn.close()
        return dict(row) if row else None

    def get_all_objects(self, exclude_removed: bool = True) -> List[dict]:
        """Get all objects."""
        conn = self._conn()
        if exclude_removed:
            rows = conn.execute(
                "SELECT * FROM objects WHERE lifecycle_state != 'REMOVED'"
            ).fetchall()
        else:
            rows = conn.execute("SELECT * FROM objects").fetchall()
        conn.close()
        return [dict(r) for r in rows]

    def get_objects_by_class(self, class_name: str) -> List[dict]:
        """Get all objects of a given class."""
        conn = self._conn()
        rows = conn.execute(
            "SELECT * FROM objects WHERE class_name=? "
            "AND lifecycle_state != 'REMOVED'",
            (class_name,)
        ).fetchall()
        conn.close()
        return [dict(r) for r in rows]

    def update_lifecycle(self, obj_id: int, new_state: str):
        """Manually update lifecycle state."""
        conn = self._conn()
        conn.execute(
            "UPDATE objects SET lifecycle_state=?, last_seen_ts=? WHERE id=?",
            (new_state, self._now_iso(), obj_id))
        conn.commit()
        conn.close()

    def remove_object(self, obj_id: int):
        """Mark object as REMOVED."""
        self.update_lifecycle(obj_id, LIFECYCLE_REMOVED)
        # Also remove its spatial relations
        conn = self._conn()
        conn.execute(
            "DELETE FROM spatial_relations WHERE subject_id=? OR object_id=?",
            (obj_id, obj_id))
        conn.commit()
        conn.close()

    # ═══════════════════════════════════════════════════════
    # Lifecycle Tick (call at 30Hz from VisionAgent)
    # ═══════════════════════════════════════════════════════
    def tick_unseen(self, seen_ids: set):
        """
        Increment unseen counters for objects not in seen_ids.
        Transitions: TRACKED→LOST after 30 frames, LOST→REMOVED after 300.
        """
        conn = self._conn()
        active = conn.execute(
            "SELECT id, lifecycle_state FROM objects "
            "WHERE lifecycle_state IN ('DETECTED','TRACKED','RECOVERED','MOVED')"
        ).fetchall()
        conn.close()

        for row in active:
            oid = row["id"]
            if oid not in seen_ids:
                self._frames_since_seen[oid] = \
                    self._frames_since_seen.get(oid, 0) + 1
                frames = self._frames_since_seen[oid]

                if frames >= REMOVED_FRAMES:
                    self.update_lifecycle(oid, LIFECYCLE_REMOVED)
                elif frames >= LOST_FRAMES:
                    current = row["lifecycle_state"]
                    if current != LIFECYCLE_LOST:
                        self.update_lifecycle(oid, LIFECYCLE_LOST)

    # ═══════════════════════════════════════════════════════
    # Spatial Relations
    # ═══════════════════════════════════════════════════════
    def update_spatial_relations(self, subject_id: int):
        """Recompute all spatial relations for a given object."""
        subject = self.get_object(subject_id)
        if subject is None:
            return

        all_objects = self.get_all_objects()
        conn = self._conn()
        now = self._now_iso()

        # Clear old relations for this subject
        conn.execute(
            "DELETE FROM spatial_relations WHERE subject_id=?",
            (subject_id,))

        sx, sy, sz = subject["pos_x"], subject["pos_y"], subject["pos_z"]

        for other in all_objects:
            if other["id"] == subject_id:
                continue

            ox, oy, oz = other["pos_x"], other["pos_y"], other["pos_z"]
            dx, dy, dz = ox - sx, oy - sy, oz - sz
            dist = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)
            horiz = math.sqrt(dx ** 2 + dy ** 2)

            relations = []

            # is_near
            if dist < NEAR_THRESHOLD_M:
                relations.append(("is_near", dist, 0.9))

            # is_left_of / is_right_of (Y axis in robot frame)
            if abs(dy) > 0.03:
                if dy > 0:
                    relations.append(("is_left_of", abs(dy), 0.8))
                else:
                    relations.append(("is_right_of", abs(dy), 0.8))

            # is_in_front_of / is_behind (X axis)
            if abs(dx) > 0.03:
                if dx > 0:
                    relations.append(("is_behind", abs(dx), 0.8))
                else:
                    relations.append(("is_in_front_of", abs(dx), 0.8))

            # is_on (B is on A: B slightly above A, horizontally close)
            if 0.0 < dz < ON_VERTICAL_M and horiz < ON_HORIZONTAL_M:
                relations.append(("is_on", dz, 0.9))

            # is_above
            if dz > ON_VERTICAL_M:
                relations.append(("is_above", dz, 0.85))

            # is_inside (close in all axes, other is larger)
            if horiz < INSIDE_THRESHOLD_M and abs(dz) < INSIDE_THRESHOLD_M:
                relations.append(("is_inside", dist, 0.7))

            for rel_type, rel_dist, conf in relations:
                conn.execute("""
                    INSERT OR REPLACE INTO spatial_relations
                        (subject_id, relation_type, object_id,
                         distance_m, confidence, last_updated)
                    VALUES (?, ?, ?, ?, ?, ?)
                """, (subject_id, rel_type, other["id"],
                      rel_dist, conf, now))

        conn.commit()
        conn.close()

    def update_all_relations(self):
        """Recompute spatial relations for all active objects."""
        for obj in self.get_all_objects():
            self.update_spatial_relations(obj["id"])

    def get_related_objects(self, obj_id: int,
                           relation_type: str = "") -> List[dict]:
        """Get objects related to a given object."""
        conn = self._conn()
        if relation_type:
            rows = conn.execute("""
                SELECT sr.*, o.class_name, o.display_name, o.pos_x, o.pos_y, o.pos_z
                FROM spatial_relations sr
                JOIN objects o ON sr.object_id = o.id
                WHERE sr.subject_id=? AND sr.relation_type=?
            """, (obj_id, relation_type)).fetchall()
        else:
            rows = conn.execute("""
                SELECT sr.*, o.class_name, o.display_name, o.pos_x, o.pos_y, o.pos_z
                FROM spatial_relations sr
                JOIN objects o ON sr.object_id = o.id
                WHERE sr.subject_id=?
            """, (obj_id,)).fetchall()
        conn.close()
        return [dict(r) for r in rows]

    def get_all_relations(self) -> List[dict]:
        """Get all spatial relations."""
        conn = self._conn()
        rows = conn.execute("""
            SELECT sr.*,
                   s.display_name as subject_name,
                   o.display_name as object_name
            FROM spatial_relations sr
            JOIN objects s ON sr.subject_id = s.id
            JOIN objects o ON sr.object_id = o.id
        """).fetchall()
        conn.close()
        return [dict(r) for r in rows]

    # ═══════════════════════════════════════════════════════
    # Task History
    # ═══════════════════════════════════════════════════════
    def log_task(self, command: str, success: bool,
                 duration_s: float, failure_reason: str = ""):
        """Log a completed task."""
        conn = self._conn()
        conn.execute("""
            INSERT INTO task_history
                (command, success, duration_s, failure_reason, timestamp)
            VALUES (?, ?, ?, ?, ?)
        """, (command, 1 if success else 0, duration_s,
              failure_reason, self._now_iso()))
        conn.commit()
        conn.close()

    def get_task_history(self, limit: int = 100) -> List[dict]:
        """Get recent task history."""
        conn = self._conn()
        rows = conn.execute(
            "SELECT * FROM task_history ORDER BY id DESC LIMIT ?",
            (limit,)).fetchall()
        conn.close()
        return [dict(r) for r in rows]

    def get_task_stats(self) -> dict:
        """Get aggregate task statistics."""
        conn = self._conn()
        total = conn.execute(
            "SELECT COUNT(*) FROM task_history").fetchone()[0]
        success = conn.execute(
            "SELECT COUNT(*) FROM task_history WHERE success=1"
        ).fetchone()[0]
        avg_dur = conn.execute(
            "SELECT AVG(duration_s) FROM task_history"
        ).fetchone()[0] or 0.0
        conn.close()
        return {
            "total_tasks": total,
            "successful": success,
            "failed": total - success,
            "success_rate": success / max(total, 1),
            "avg_duration_s": avg_dur,
        }

    # ═══════════════════════════════════════════════════════
    # YAML Import / Export
    # ═══════════════════════════════════════════════════════
    def export_to_yaml(self, path: str):
        """Export entire world model to YAML for debugging."""
        data = {
            "objects": self.get_all_objects(exclude_removed=False),
            "relations": self.get_all_relations(),
            "task_stats": self.get_task_stats(),
            "exported_at": self._now_iso(),
        }
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        with open(path, "w") as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False)

    def import_from_yaml(self, path: str):
        """Import world model from YAML (for loading pre-mapped workspaces)."""
        with open(path, "r") as f:
            data = yaml.safe_load(f)

        conn = self._conn()

        for obj in data.get("objects", []):
            conn.execute("""
                INSERT OR REPLACE INTO objects
                    (id, class_name, display_name,
                     pos_x, pos_y, pos_z,
                     rot_x, rot_y, rot_z, rot_w,
                     color, material,
                     size_x, size_y, size_z,
                     lifecycle_state, last_seen_ts,
                     detection_count, affordance_scores)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?,
                        ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """, (
                obj.get("id"), obj.get("class_name", ""),
                obj.get("display_name", ""),
                obj.get("pos_x", 0), obj.get("pos_y", 0), obj.get("pos_z", 0),
                obj.get("rot_x", 0), obj.get("rot_y", 0),
                obj.get("rot_z", 0), obj.get("rot_w", 1),
                obj.get("color", ""), obj.get("material", ""),
                obj.get("size_x", 0.04), obj.get("size_y", 0.04),
                obj.get("size_z", 0.04),
                obj.get("lifecycle_state", "DETECTED"),
                obj.get("last_seen_ts", self._now_iso()),
                obj.get("detection_count", 0),
                json.dumps(obj.get("affordance_scores", {})),
            ))

        conn.commit()
        conn.close()

        # Recompute all spatial relations
        self.update_all_relations()

    # ═══════════════════════════════════════════════════════
    # Utility
    # ═══════════════════════════════════════════════════════
    def object_count(self) -> int:
        conn = self._conn()
        n = conn.execute(
            "SELECT COUNT(*) FROM objects WHERE lifecycle_state != 'REMOVED'"
        ).fetchone()[0]
        conn.close()
        return n

    def clear(self):
        """Clear all data (for testing)."""
        conn = self._conn()
        conn.executescript("""
            DELETE FROM objects;
            DELETE FROM spatial_relations;
            DELETE FROM task_history;
        """)
        conn.commit()
        conn.close()
        self._consecutive_frames.clear()
        self._frames_since_seen.clear()
        self._last_positions.clear()
