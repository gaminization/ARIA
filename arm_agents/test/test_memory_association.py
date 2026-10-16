import tempfile
import time
import unittest

import rclpy
from arm_planner.msg import ObjectDetection, VisionState, MemoryState, WorldObject

from arm_agents.memory_agent import MemoryAgent


class MemoryAssociationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.agent = MemoryAgent()
        self.agent.db_path = f"{self.temp_dir.name}/world_model.db"
        self.agent.bus.add_chain_of_thought = lambda _text: None
        self.agent._init_db()

    def tearDown(self):
        self.agent.destroy_node()
        self.temp_dir.cleanup()

    @staticmethod
    def detection(track_id, class_name, x, y, bbox_x, bbox_y, color="blue"):
        det = ObjectDetection()
        det.tracking_id = track_id
        det.class_name = class_name
        det.pose_3d.pose.position.x = x
        det.pose_3d.pose.position.y = y
        det.pose_3d.pose.position.z = 0.65
        det.pose_3d.pose.orientation.w = 1.0
        det.bbox_x = float(bbox_x)
        det.bbox_y = float(bbox_y)
        det.bbox_w = 30.0
        det.bbox_h = 30.0
        det.affordance_regions = [f"color:{color}"]
        return det

    def send_frame(self, detections):
        state = VisionState()
        state.detected_objects = detections
        self.agent._on_vision(state)

    def test_tracks_distinct_same_class_objects_four_point_five_cm_apart(self):
        for _ in range(3):
            self.send_frame([
                self.detection(101, "workpiece", 0.20, 0.12, 100, 100),
                self.detection(102, "workpiece", 0.20, 0.165, 150, 100),
            ])

        self.assertEqual(len(self.agent.objects), 2)
        first_id = self.agent.track_to_object[101]
        second_id = self.agent.track_to_object[102]
        self.assertNotEqual(first_id, second_id)

        self.send_frame([
            self.detection(101, "workpiece", 0.20, 0.14, 100, 100),
            self.detection(102, "workpiece", 0.20, 0.185, 150, 100),
        ])

        first_y = self.agent.objects[first_id].last_known_pose.pose.position.y
        second_y = self.agent.objects[second_id].last_known_pose.pose.position.y
        self.assertGreater(abs(second_y - first_y), 0.035)
        self.assertEqual(len(self.agent.objects), 2)

    def test_deduplicates_overlapping_alias_detections(self):
        for _ in range(3):
            self.send_frame([
                self.detection(201, "cube", 0.20, 0.12, 100, 100, color="red"),
                self.detection(202, "block", 0.20, 0.12, 100, 100, color="red"),
            ])

        self.assertEqual(len(self.agent.objects), 1)

    def test_preserves_external_held_state_and_retains_object(self):
        for _ in range(3):
            self.send_frame([
                self.detection(301, "workpiece", 0.20, 0.12, 100, 100),
            ])

        object_id = self.agent.track_to_object[301]
        obj = self.agent.objects[object_id]
        external = WorldObject()
        external.id = obj.id + 1000
        external.name = "workpiece_target"
        external.class_name = obj.class_name
        external.last_known_pose = obj.last_known_pose
        external.lifecycle_state = "Held"
        state = MemoryState()
        state.known_objects = [external]
        self.agent._on_external_memory(state)

        self.send_frame([
            self.detection(301, "workpiece", 0.20, 0.14, 100, 100),
        ])
        self.assertEqual(obj.lifecycle_state, "Held")

        obj.last_seen.sec = int(time.time()) - 10
        self.agent._publish_state()
        self.assertIn(object_id, self.agent.objects)

        external.lifecycle_state = "Tracked"
        self.agent._on_external_memory(state)
        self.assertEqual(obj.lifecycle_state, "Held")
        external.lifecycle_state = "Released"
        self.agent._on_external_memory(state)
        self.assertEqual(obj.lifecycle_state, "Tracked")

    def test_imports_and_tracks_held_object_missing_from_local_model(self):
        external = WorldObject()
        external.id = 501
        external.name = "workpiece_target"
        external.class_name = "workpiece"
        external.lifecycle_state = "Held"
        external.last_known_pose.pose.position.x = 0.20
        external.last_known_pose.pose.position.y = 0.20
        external.last_known_pose.pose.position.z = 0.65
        state = MemoryState()
        state.known_objects = [external]

        self.agent._on_external_memory(state)
        self.assertEqual(len(self.agent.objects), 1)
        local_object = next(iter(self.agent.objects.values()))
        self.assertEqual(local_object.lifecycle_state, "Held")

        external.lifecycle_state = "Released"
        self.agent._on_external_memory(state)
        self.assertEqual(local_object.lifecycle_state, "Tracked")
        self.assertEqual(len(self.agent.objects), 1)

    def test_merges_external_detection_without_replacing_world_model(self):
        for _ in range(3):
            self.send_frame([
                self.detection(601, "workpiece", 0.20, 0.12, 100, 100),
            ])
        local_object = next(iter(self.agent.objects.values()))

        external = WorldObject()
        external.id = 900
        external.name = "scan_workpiece"
        external.class_name = "workpiece"
        external.lifecycle_state = "Tracked"
        external.last_known_pose = local_object.last_known_pose
        state = MemoryState()
        state.known_objects = [external]
        self.agent._on_external_memory(state)

        self.assertEqual(len(self.agent.objects), 1)
        self.assertEqual(local_object.lifecycle_state, "Tracked")


if __name__ == "__main__":
    unittest.main()
