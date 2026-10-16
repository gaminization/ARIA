import unittest
import time

from arm_vision.detection_node import DetectionNode


class DetectionTrackingTests(unittest.TestCase):
    def test_suppresses_overlaps_without_dropping_close_objects(self):
        close_boxes = [
            (0.0, 0.0, 20.0, 20.0, 0.95, "workpiece", "blue", None),
            (21.0, 0.0, 41.0, 20.0, 0.90, "workpiece", "blue", None),
        ]
        self.assertEqual(len(DetectionNode._suppress_duplicate_boxes(close_boxes)), 2)

        duplicate_boxes = [
            (0.0, 0.0, 20.0, 20.0, 0.95, "workpiece", "blue", None),
            (1.0, 1.0, 21.0, 21.0, 0.90, "workpiece", "blue", None),
        ]
        self.assertEqual(len(DetectionNode._suppress_duplicate_boxes(duplicate_boxes)), 1)

    def test_track_ids_are_unique_per_frame_and_remain_stable(self):
        node = DetectionNode.__new__(DetectionNode)
        node.active_tracks = {}
        node.next_track_id = 1

        first_frame_ids = set()
        first_id = node._assign_frame_track_id(
            100.0, 100.0, "workpiece|blue", None, first_frame_ids
        )
        second_id = node._assign_frame_track_id(
            110.0, 100.0, "workpiece|blue", None, first_frame_ids
        )
        self.assertNotEqual(first_id, second_id)

        next_frame_ids = set()
        self.assertEqual(
            node._assign_frame_track_id(
                102.0, 100.0, "workpiece|blue", None, next_frame_ids
            ),
            first_id,
        )
        self.assertEqual(
            node._assign_frame_track_id(
                112.0, 100.0, "workpiece|blue", None, next_frame_ids
            ),
            second_id,
        )

    def test_does_not_reuse_a_neighbor_object_track(self):
        node = DetectionNode.__new__(DetectionNode)
        node.active_tracks = {
        7: (100.0, 100.0, "workpiece|blue", time.time()),
        }
        node.next_track_id = 8

        track_id = node._get_persistent_track_id(
        154.0, 100.0, "workpiece|blue", set()
        )
        self.assertNotEqual(track_id, 7)


if __name__ == "__main__":
    unittest.main()
