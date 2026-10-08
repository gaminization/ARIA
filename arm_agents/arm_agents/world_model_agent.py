#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA World Model Agent — LifecycleNode
3D scene understanding. Spatial relations. RViz visualization.
═══════════════════════════════════════════════════════════════
"""
import math
import numpy as np
import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from arm_planner.msg import MemoryState, SemanticRelation, WorldObject
from arm_planner.state_bus import StateBus

# Spatial relation thresholds
NEAR_THRESHOLD_M = 0.10
LEFT_RIGHT_ANGLE_DEG = 45.0
TABLE_EDGE_MARGIN_M = 0.05

# Table dimensions from SDF
TABLE_X_MIN, TABLE_X_MAX = -0.30, 0.30
TABLE_Y_MIN, TABLE_Y_MAX = -0.30, 0.30
TABLE_Z = 0.76

class WorldModelAgent(LifecycleNode):
    """
    Maintains semantic 3D scene understanding.

    Spatial relations tracked:
      is_left_of, is_right_of, is_in_front_of, is_behind
      is_on, is_inside, is_near, is_near_edge

    Publishes:
      /world_model/objects  — bounding boxes in RViz
      /world_model/relations — lines between related objects
    """

    def __init__(self):
        super().__init__('world_model_agent')
        self.bus = StateBus(self)
        self.relations: list = []

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("WorldModelAgent: CONFIGURING")
        self.bus.on_change('memory', self._on_memory)
        self.obj_viz_pub = self.create_publisher(
            MarkerArray, '/world_model/objects', 10)
        self.rel_viz_pub = self.create_publisher(
            MarkerArray, '/world_model/relations', 10)
        self.create_timer(0.5, self._publish_viz)  # 2Hz
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("WorldModelAgent: ACTIVATED")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def _on_memory(self, msg: MemoryState):
        """Recompute spatial relations when memory updates."""
        objects = msg.known_objects
        self.relations.clear()

        for i, a in enumerate(objects):
            ax = a.last_known_pose.pose.position.x
            ay = a.last_known_pose.pose.position.y
            az = a.last_known_pose.pose.position.z

            # Edge proximity
            dist_to_edge = min(
                abs(ax - TABLE_X_MIN), abs(ax - TABLE_X_MAX),
                abs(ay - TABLE_Y_MIN), abs(ay - TABLE_Y_MAX))
            if dist_to_edge < TABLE_EDGE_MARGIN_M:
                self.relations.append(self._make_relation(
                    a.name, 'is_near_edge', 'table', 0.9, dist_to_edge))

            for j, b in enumerate(objects):
                if i == j:
                    continue
                bx = b.last_known_pose.pose.position.x
                by = b.last_known_pose.pose.position.y
                bz = b.last_known_pose.pose.position.z

                dx, dy, dz = bx - ax, by - ay, bz - az
                dist = math.sqrt(dx**2 + dy**2 + dz**2)

                # Near
                if dist < NEAR_THRESHOLD_M:
                    self.relations.append(self._make_relation(
                        a.name, 'is_near', b.name, 0.9, dist))

                # Left/right (Y axis)
                if abs(dy) > 0.03:
                    if dy > 0:
                        self.relations.append(self._make_relation(
                            a.name, 'is_left_of', b.name, 0.8, abs(dy)))
                    else:
                        self.relations.append(self._make_relation(
                            a.name, 'is_right_of', b.name, 0.8, abs(dy)))

                # Above/below
                if abs(dz) > 0.02:
                    if dz > 0:
                        self.relations.append(self._make_relation(
                            a.name, 'is_above', b.name, 0.85, abs(dz)))

                # Is on (very close vertically, almost touching)
                if 0.0 < dz < 0.05 and math.sqrt(dx**2 + dy**2) < 0.03:
                    self.relations.append(self._make_relation(
                        b.name, 'is_on', a.name, 0.9, dz))

    def _make_relation(self, subject, relation, obj, confidence, distance):
        r = SemanticRelation()
        r.subject = subject
        r.relation = relation
        r.object = obj
        r.confidence = confidence
        r.distance_m = float(distance)
        return r

    def _publish_viz(self):
        """Publish RViz visualization."""
        memory = self.bus.state.memory
        if memory is None:
            return

        # Object markers (cubes)
        obj_markers = MarkerArray()
        for i, wo in enumerate(memory.known_objects):
            m = Marker()
            m.header.frame_id = 'world'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'world_objects'
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose = wo.last_known_pose.pose
            m.scale.x = m.scale.y = m.scale.z = 0.04
            m.color.r, m.color.g, m.color.b = 0.2, 0.8, 0.2
            m.color.a = 0.6
            m.lifetime.sec = 2
            # Text label
            txt = Marker()
            txt.header = m.header
            txt.ns = 'world_labels'
            txt.id = i + 1000
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose = wo.last_known_pose.pose
            txt.pose.position.z += 0.06
            txt.text = wo.name
            txt.scale.z = 0.02
            txt.color.r = txt.color.g = txt.color.b = 1.0
            txt.color.a = 1.0
            txt.lifetime.sec = 2
            obj_markers.markers.extend([m, txt])
        self.obj_viz_pub.publish(obj_markers)

        # Relation markers (lines)
        rel_markers = MarkerArray()
        name_to_pose = {wo.name: wo.last_known_pose.pose for wo in memory.known_objects}
        for i, rel in enumerate(self.relations):
            if rel.subject in name_to_pose and rel.object in name_to_pose:
                m = Marker()
                m.header.frame_id = 'world'
                m.header.stamp = self.get_clock().now().to_msg()
                m.ns = 'world_relations'
                m.id = i
                m.type = Marker.LINE_STRIP
                m.action = Marker.ADD
                m.scale.x = 0.003
                p1 = Point()
                p1.x = name_to_pose[rel.subject].position.x
                p1.y = name_to_pose[rel.subject].position.y
                p1.z = name_to_pose[rel.subject].position.z
                p2 = Point()
                p2.x = name_to_pose[rel.object].position.x
                p2.y = name_to_pose[rel.object].position.y
                p2.z = name_to_pose[rel.object].position.z
                m.points = [p1, p2]
                m.color.r, m.color.g, m.color.b = 0.5, 0.5, 1.0
                m.color.a = 0.5
                m.lifetime.sec = 2
                rel_markers.markers.append(m)
        self.rel_viz_pub.publish(rel_markers)

def main(args=None):
    rclpy.init(args=args)
    node = WorldModelAgent()
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
