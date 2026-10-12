"""
ARIA Semantic Map — Dedicated Scene Graph & Spatial Reasoning Node
===================================================================

Surfaces a queryable semantic map over ARIA's physical workspace.
Computes and maintains directed semantic spatial relations between
detected objects (e.g. "cup is_left_of bottle", "block_a is_on block_b").

Provides both:
  1. SemanticMap: Pure-Python queryable class wrapping WorldModelDB & scene graphs
  2. SemanticMapNode: ROS 2 lifecycle/standard node publishing relations to /aria/state/semantic_map
"""

import math
import os
import sqlite3
from typing import Any, Dict, List, Optional, Tuple, Union

try:
    from arm_planner.msg import SemanticRelation
except ImportError:
    from dataclasses import dataclass

    @dataclass
    class SemanticRelation:
        subject: str = ""
        relation: str = ""
        object: str = ""
        confidence: float = 1.0
        distance_m: float = 0.0


class SemanticMap:
    """
    Queryable Semantic Map and Scene Graph for ARIA arm manipulation.
    """

    RELATIONS = [
        "left_of",
        "right_of",
        "in_front_of",
        "behind",
        "on_top_of",
        "under",
        "near",
        "inside",
    ]

    def __init__(self, db_path: str = ""):
        from arm_planner.world_model_db import WorldModelDB
        self.db = WorldModelDB(db_path=db_path)
        self._cache_relations: List[Dict[str, Any]] = []

    def refresh(self):
        """Recompute all spatial relations in the underlying world model."""
        self.db.update_all_relations()
        self._cache_relations = self.db.get_all_relations()

    def get_all_relations(self) -> List[Dict[str, Any]]:
        """Return all active spatial relations."""
        self.refresh()
        return self._cache_relations

    def get_relation_messages(self) -> List[SemanticRelation]:
        """Convert all active relations to ROS SemanticRelation messages."""
        self.refresh()
        msgs = []
        for r in self._cache_relations:
            msg = SemanticRelation()
            msg.subject = str(r.get("subject_name", ""))
            msg.relation = str(r.get("relation_type", ""))
            msg.object = str(r.get("object_name", ""))
            msg.distance_m = float(r.get("distance_m", 0.0))
            msg.confidence = float(r.get("confidence", 1.0))
            msgs.append(msg)
        return msgs

    def get_relations_between(self, subject_name: str, object_name: str) -> List[str]:
        """Find all relation types between two named objects."""
        self.refresh()
        sub = subject_name.lower().strip()
        obj = object_name.lower().strip()
        relations = []
        for r in self._cache_relations:
            r_sub = str(r.get("subject_name", "")).lower()
            r_obj = str(r.get("object_name", "")).lower()
            if (sub in r_sub or r_sub in sub) and (obj in r_obj or r_obj in obj):
                relations.append(r.get("relation_type", ""))
        return relations

    def find_objects(self, relation: str, reference_object: str) -> List[Dict[str, Any]]:
        """
        Find objects that satisfy (X, relation, reference_object).
        Example: find_objects("left_of", "blue_cube") -> returns all objects left of blue_cube.
        """
        self.refresh()
        ref = reference_object.lower().strip()
        rel = relation.lower().strip().replace(" ", "_")
        matches = []
        for r in self._cache_relations:
            r_obj = str(r.get("object_name", "")).lower()
            r_rel = str(r.get("relation_type", "")).lower()
            if (ref in r_obj or r_obj in ref) and (rel == r_rel or rel in r_rel):
                sub_id = r.get("subject_id")
                obj_data = self.db.get_object(sub_id) if sub_id else None
                matches.append({
                    "name": r.get("subject_name"),
                    "id": sub_id,
                    "distance_m": r.get("distance_m", 0.0),
                    "confidence": r.get("confidence", 1.0),
                    "object_info": obj_data
                })
        return matches

    def query(self, query_str: str) -> Dict[str, Any]:
        """
        Query the semantic map with natural language fragments or structured tuples.
        Supports phrases like:
          - "what is left of bottle"
          - "what is on the table"
          - "is cup near bowl"
        """
        q = query_str.lower().strip()
        self.refresh()

        # Check for relation keywords
        for rel in ["left_of", "right_of", "in_front_of", "behind", "on_top_of", "near", "inside"]:
            words_rel = rel.replace("_", " ")
            if words_rel in q or rel in q:
                # Find target object after relation
                after = q.split(words_rel)[-1] if words_rel in q else q.split(rel)[-1]
                after = after.replace("the", "").replace("a", "").strip(" ?.")
                if after:
                    results = self.find_objects(rel, after)
                    return {
                        "query": query_str,
                        "relation": rel,
                        "reference": after,
                        "found_objects": results,
                        "success": len(results) > 0
                    }

        # General scene graph summary
        return {
            "query": query_str,
            "scene_graph": self.get_scene_graph(),
            "total_objects": self.db.object_count(),
            "success": True
        }

    def get_scene_graph(self) -> Dict[str, Any]:
        """
        Construct a full directed semantic scene graph of the workspace.
        """
        self.refresh()
        objects = self.db.get_all_objects(exclude_removed=True)
        nodes = []
        for obj in objects:
            nodes.append({
                "id": obj.get("id"),
                "class_name": obj.get("class_name"),
                "display_name": obj.get("display_name"),
                "position": [obj.get("pos_x", 0.0), obj.get("pos_y", 0.0), obj.get("pos_z", 0.0)],
                "color": obj.get("color", ""),
                "material": obj.get("material", ""),
                "lifecycle_state": obj.get("lifecycle_state", "")
            })

        edges = []
        for r in self._cache_relations:
            edges.append({
                "subject_id": r.get("subject_id"),
                "subject": r.get("subject_name"),
                "relation": r.get("relation_type"),
                "object_id": r.get("object_id"),
                "object": r.get("object_name"),
                "distance_m": r.get("distance_m", 0.0),
                "confidence": r.get("confidence", 1.0)
            })

        return {
            "nodes": nodes,
            "edges": edges,
            "node_count": len(nodes),
            "edge_count": len(edges)
        }


# ═══════════════════════════════════════════════════════════════
# Optional ROS 2 Node Integration
# ═══════════════════════════════════════════════════════════════
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    import json

    class SemanticMapNode(Node):
        """ROS 2 Node publishing semantic map updates and providing queries."""

        def __init__(self, node_name: str = "semantic_map_node"):
            super().__init__(node_name)
            self.map = SemanticMap()
            self.pub_scene_graph = self.create_publisher(
                String, "/aria/semantic_map/scene_graph", 10
            )
            self.pub_timer = self.create_timer(2.0, self._publish_map)
            self.get_logger().info("SemanticMapNode initialized.")

        def _publish_map(self):
            graph = self.map.get_scene_graph()
            msg = String()
            msg.data = json.dumps(graph)
            self.pub_scene_graph.publish(msg)

except ImportError:
    SemanticMapNode = None
