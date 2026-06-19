#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Memory Agent — LifecycleNode
Persistent object storage. SQLite-backed WorldModel.
Publishes MemoryState to state bus at 2Hz.
═══════════════════════════════════════════════════════════════
"""
import os, time, sqlite3
from typing import Dict, Optional
import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from geometry_msgs.msg import PoseStamped
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

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("MemoryAgent: CONFIGURING")
        self.declare_parameter('db_path', '')
        db_path = self.get_parameter('db_path').value
        if not db_path:
            db_path = os.path.join(
                os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                '..', 'arm_planner', 'data', 'world_model.db')
        self.db_path = db_path
        self._init_db()
        self._load_from_db()
        self.bus.on_change('vision', self._on_vision)
        self.create_timer(0.5, self._publish_state)  # 2Hz
        return TransitionCallbackReturn.SUCCESS

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

    def _save_all(self):
        for wo in self.objects.values():
            self._save_object(wo)

    def _on_vision(self, msg: VisionState):
        """Update memory from vision detections."""
        for det in msg.detected_objects:
            if det.tracking_id < 0:
                continue

            # Check if we know this object
            known = None
            for wo in self.objects.values():
                if wo.class_name == det.class_name:
                    # Check if position is close enough to be same object
                    if det.pose_3d.pose.position.x != 0:
                        dx = abs(wo.last_known_pose.pose.position.x -
                                 det.pose_3d.pose.position.x)
                        dy = abs(wo.last_known_pose.pose.position.y -
                                 det.pose_3d.pose.position.y)
                        if dx < 0.1 and dy < 0.1:
                            known = wo
                            break

            if known is None:
                # New object
                wo = WorldObject()
                wo.id = self.next_id
                self.next_id += 1
                wo.name = f"{det.class_name}_{wo.id}"
                wo.class_name = det.class_name
                wo.last_known_pose = det.pose_3d
                wo.lifecycle_state = 'Detected'
                wo.color = ''
                wo.material = ''
                self.objects[wo.id] = wo
                self._save_object(wo)
                self.bus.add_chain_of_thought(
                    f"MEMORY: New object registered: {wo.name} (ID={wo.id})")
            else:
                # Update existing
                known.last_known_pose = det.pose_3d
                known.lifecycle_state = 'Tracked'
                self._save_object(known)

    def _publish_state(self):
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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
