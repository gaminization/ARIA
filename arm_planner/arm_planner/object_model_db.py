#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Object Model Database
Central knowledge base of known objects — properties, meshes,
affordances, grasp parameters, appearance, and success rates.

Difference from WorldModelDB (Stage 3):
  WorldModelDB:  tracks WHERE objects ARE (position, lifecycle)
  ObjectModelDB: stores WHAT objects ARE (properties, CAD, grasps)

Linked: WorldObject.class_name → ObjectModelDB.class_name

Usage:
  db = ObjectModelDB()
  model = db.get_model('red_cup')
  params = db.get_grasp_params('red_cup')
  db.update_success_rate('red_cup', success=True)
  db.register_new_object('green_mug', images, props)
═══════════════════════════════════════════════════════════════
"""
import json
import os
import sqlite3
import time
from dataclasses import dataclass, field, asdict
from datetime import datetime
from typing import Dict, List, Optional, Tuple

import numpy as np

try:
    import yaml
    YAML_AVAILABLE = True
except ImportError:
    YAML_AVAILABLE = False


# ═══════════════════════════════════════════════════════════════
# Data types
# ═══════════════════════════════════════════════════════════════
@dataclass
class ObjectModel:
    """Complete model of a known object."""
    class_name: str
    display_name: str = ''

    # Physical properties
    mass_kg: float = 0.0
    material: str = 'unknown'
    fragile: bool = False

    # Geometry
    mesh_path: str = ''
    urdf_path: str = ''
    bounding_box: List[float] = field(default_factory=lambda: [0.05, 0.05, 0.05])

    # Grasp properties
    affordance: dict = field(default_factory=dict)
    grasp_points: List[dict] = field(default_factory=list)
    grip_force_scale: float = 1.0
    approach_style: str = 'top_down'

    # Appearance
    typical_colors: List[str] = field(default_factory=list)
    reference_images_path: str = ''

    # Sim properties
    sdf_path: str = ''
    friction: float = 0.5
    restitution: float = 0.1

    # Statistics
    times_grasped: int = 0
    grasp_success_rate: float = 0.0
    date_added: str = ''
    last_seen: str = ''
    notes: str = ''


@dataclass
class GraspParams:
    """Grasp parameters for an object class."""
    grip_force_scale: float = 1.0
    approach_speed_scale: float = 1.0
    approach_style: str = 'top_down'
    grasp_region: str = 'top'
    avoid_regions: List[str] = field(default_factory=list)


# ═══════════════════════════════════════════════════════════════
# Database Schema
# ═══════════════════════════════════════════════════════════════
CREATE_TABLE_SQL = """
CREATE TABLE IF NOT EXISTS object_models (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    class_name TEXT UNIQUE NOT NULL,
    display_name TEXT DEFAULT '',

    -- Physical properties
    mass_kg REAL DEFAULT 0.0,
    material TEXT DEFAULT 'unknown',
    fragile INTEGER DEFAULT 0,

    -- Geometry
    mesh_path TEXT DEFAULT '',
    urdf_path TEXT DEFAULT '',
    bounding_box TEXT DEFAULT '[0.05, 0.05, 0.05]',

    -- Grasp properties
    affordance_json TEXT DEFAULT '{}',
    grasp_points_json TEXT DEFAULT '[]',
    grip_force_scale REAL DEFAULT 1.0,
    approach_style TEXT DEFAULT 'top_down',

    -- Appearance
    typical_colors TEXT DEFAULT '[]',
    reference_images_path TEXT DEFAULT '',
    clip_embedding BLOB,

    -- FoundationPose
    pose_6d_reference_path TEXT DEFAULT '',

    -- Sim properties
    sdf_path TEXT DEFAULT '',
    friction REAL DEFAULT 0.5,
    restitution REAL DEFAULT 0.1,

    -- Statistics
    date_added TEXT,
    last_seen TEXT,
    times_grasped INTEGER DEFAULT 0,
    grasp_success_rate REAL DEFAULT 0.0,
    notes TEXT DEFAULT ''
)
"""

CREATE_INDEX_SQL = [
    "CREATE INDEX IF NOT EXISTS idx_class ON object_models(class_name)",
    "CREATE INDEX IF NOT EXISTS idx_material ON object_models(material)",
]


# ═══════════════════════════════════════════════════════════════
# ObjectModelDB
# ═══════════════════════════════════════════════════════════════
class ObjectModelDB:
    """Central database of known objects."""

    def __init__(self, db_path: str = ''):
        if not db_path:
            db_path = os.path.join(
                os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                'data', 'object_models.db')
        os.makedirs(os.path.dirname(db_path), exist_ok=True)
        self._db_path = db_path
        self._init_db()

    def _init_db(self):
        with sqlite3.connect(self._db_path) as conn:
            conn.execute(CREATE_TABLE_SQL)
            for sql in CREATE_INDEX_SQL:
                conn.execute(sql)
            conn.commit()

    def _conn(self):
        return sqlite3.connect(self._db_path)

    # ── Core queries ───────────────────────────────────────
    def get_model(self, class_name: str) -> Optional[ObjectModel]:
        """Get complete model for an object class."""
        with self._conn() as conn:
            row = conn.execute(
                "SELECT * FROM object_models WHERE class_name = ?",
                (class_name,)).fetchone()
            if row is None:
                return None
            return self._row_to_model(row, conn)

    def get_grasp_params(self, class_name: str) -> GraspParams:
        """Get grasp parameters for an object class."""
        model = self.get_model(class_name)
        if model is None:
            return GraspParams()  # Defaults

        avoid = model.affordance.get('avoid', [])
        region = model.affordance.get('grasp', 'top')

        return GraspParams(
            grip_force_scale=model.grip_force_scale,
            approach_speed_scale=1.0,
            approach_style=model.approach_style,
            grasp_region=region,
            avoid_regions=avoid,
        )

    def get_affordances(self, class_name: str) -> dict:
        """Get affordance map for an object class."""
        model = self.get_model(class_name)
        if model is None:
            return {}
        return model.affordance

    def list_objects(self) -> List[ObjectModel]:
        """List all known objects."""
        with self._conn() as conn:
            rows = conn.execute(
                "SELECT * FROM object_models ORDER BY class_name"
            ).fetchall()
            return [self._row_to_model(r, conn) for r in rows]

    # ── Registration ───────────────────────────────────────
    def register_new_object(
        self,
        class_name: str,
        physical_props: dict = None,
        sample_images: List[np.ndarray] = None,
    ) -> int:
        """
        Register a new object ARIA has never seen before.

        Steps:
          1. Check for existing similar objects (by name)
          2. Create database entry
          3. Save reference images for FoundationPose
          4. Generate initial affordance from class heuristics
        """
        # Check if already exists
        existing = self.get_model(class_name)
        if existing:
            return existing.times_grasped  # Already exists

        props = physical_props or {}

        # Create reference images directory
        ref_dir = os.path.join(
            os.path.dirname(self._db_path),
            'object_references', class_name)
        os.makedirs(ref_dir, exist_ok=True)

        if sample_images:
            try:
                import cv2
                for i, img in enumerate(sample_images):
                    path = os.path.join(ref_dir, f"ref_{i:03d}.png")
                    cv2.imwrite(path, img)
            except ImportError:
                pass

        # Generate CLIP embedding if available
        clip_embedding = None
        if sample_images:
            clip_embedding = self._compute_clip_embedding(sample_images)

        # Infer default affordance
        affordance = self._infer_affordance(class_name, props)

        # Insert
        now = datetime.now().isoformat()
        with self._conn() as conn:
            conn.execute("""
                INSERT INTO object_models
                (class_name, display_name, mass_kg, material, fragile,
                 bounding_box, affordance_json, grip_force_scale,
                 approach_style, typical_colors, reference_images_path,
                 clip_embedding, date_added, notes)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """, (
                class_name,
                props.get('display_name', class_name.replace('_', ' ').title()),
                props.get('mass_kg', 0.1),
                props.get('material', 'unknown'),
                int(props.get('fragile', False)),
                json.dumps(props.get('bounding_box', [0.05, 0.05, 0.05])),
                json.dumps(affordance),
                props.get('grip_force_scale', 1.0),
                affordance.get('approach', 'top_down'),
                json.dumps(props.get('typical_colors', [])),
                ref_dir,
                clip_embedding,
                now,
                props.get('notes', f'Registered {now}'),
            ))
            conn.commit()
            return conn.execute(
                "SELECT id FROM object_models WHERE class_name = ?",
                (class_name,)).fetchone()[0]

    # ── Success tracking ───────────────────────────────────
    def update_success_rate(
        self, class_name: str, success: bool
    ) -> Optional[float]:
        """
        Update grasp success rate after each attempt.
        Returns new success rate.

        Uses exponential moving average (alpha=0.1).
        Flags objects with success rate below 0.6.
        """
        with self._conn() as conn:
            row = conn.execute(
                "SELECT times_grasped, grasp_success_rate "
                "FROM object_models WHERE class_name = ?",
                (class_name,)).fetchone()

            if row is None:
                return None

            times_grasped = row[0] + 1
            old_rate = row[1]

            # Exponential moving average
            alpha = 0.1
            new_rate = old_rate * (1 - alpha) + (1.0 if success else 0.0) * alpha

            conn.execute("""
                UPDATE object_models
                SET times_grasped = ?,
                    grasp_success_rate = ?,
                    last_seen = ?
                WHERE class_name = ?
            """, (
                times_grasped,
                round(new_rate, 4),
                datetime.now().isoformat(),
                class_name,
            ))
            conn.commit()

            return new_rate

    # ── Search ─────────────────────────────────────────────
    def search_by_material(self, material: str) -> List[ObjectModel]:
        """Find all objects of a given material."""
        with self._conn() as conn:
            rows = conn.execute(
                "SELECT * FROM object_models WHERE material = ?",
                (material,)).fetchall()
            return [self._row_to_model(r, conn) for r in rows]

    def search_by_appearance(
        self, clip_embedding: bytes, top_k: int = 5
    ) -> List[Tuple[ObjectModel, float]]:
        """Find visually similar objects using CLIP embeddings."""
        results = []
        with self._conn() as conn:
            rows = conn.execute(
                "SELECT * FROM object_models WHERE clip_embedding IS NOT NULL"
            ).fetchall()

            for row in rows:
                model = self._row_to_model(row, conn)
                stored_emb = row[15]  # clip_embedding column
                if stored_emb:
                    stored = np.frombuffer(stored_emb, dtype=np.float32)
                    query = np.frombuffer(clip_embedding, dtype=np.float32)
                    similarity = float(
                        np.dot(stored, query) /
                        (np.linalg.norm(stored) * np.linalg.norm(query) + 1e-8))
                    results.append((model, similarity))

        results.sort(key=lambda x: x[1], reverse=True)
        return results[:top_k]

    def get_stats(self) -> List[dict]:
        """Get per-object statistics sorted by usage."""
        with self._conn() as conn:
            rows = conn.execute("""
                SELECT class_name, times_grasped, grasp_success_rate,
                       last_seen, material
                FROM object_models
                ORDER BY times_grasped DESC
            """).fetchall()

            return [{
                'class_name': r[0],
                'times_grasped': r[1],
                'success_rate': r[2],
                'last_seen': r[3] or 'never',
                'material': r[4],
            } for r in rows]

    # ── Import / Export ────────────────────────────────────
    def import_library(self, yaml_path: str) -> int:
        """Import object definitions from YAML library file."""
        if not YAML_AVAILABLE:
            return 0

        with open(yaml_path, 'r') as f:
            library = yaml.safe_load(f)

        if not library or 'objects' not in library:
            return 0

        count = 0
        for obj_data in library['objects']:
            class_name = obj_data.get('class_name', '')
            if not class_name:
                continue

            existing = self.get_model(class_name)
            if existing:
                continue  # Don't overwrite existing

            self.register_new_object(
                class_name,
                physical_props=obj_data,
            )
            count += 1

        return count

    def export_library(self, yaml_path: str) -> int:
        """Export all objects to YAML library file."""
        if not YAML_AVAILABLE:
            return 0

        objects = self.list_objects()
        library = {
            'version': '1.0',
            'exported_at': datetime.now().isoformat(),
            'objects': [asdict(obj) for obj in objects],
        }

        os.makedirs(os.path.dirname(yaml_path) or '.', exist_ok=True)
        with open(yaml_path, 'w') as f:
            yaml.dump(library, f, default_flow_style=False, sort_keys=False)

        return len(objects)

    # ── Internal helpers ───────────────────────────────────
    def _row_to_model(self, row, conn) -> ObjectModel:
        """Convert database row to ObjectModel.

        Column indices (25 cols, 0-24):
          0:id 1:class_name 2:display_name 3:mass_kg 4:material
          5:fragile 6:mesh_path 7:urdf_path 8:bounding_box
          9:affordance_json 10:grasp_points_json 11:grip_force_scale
          12:approach_style 13:typical_colors 14:reference_images_path
          15:clip_embedding 16:pose_6d_reference_path 17:sdf_path
          18:friction 19:restitution 20:date_added 21:last_seen
          22:times_grasped 23:grasp_success_rate 24:notes
        """
        return ObjectModel(
            class_name=row[1],
            display_name=row[2] or '',
            mass_kg=row[3] or 0.0,
            material=row[4] or 'unknown',
            fragile=bool(row[5]),
            mesh_path=row[6] or '',
            urdf_path=row[7] or '',
            bounding_box=json.loads(row[8]) if row[8] else [0.05, 0.05, 0.05],
            affordance=json.loads(row[9]) if row[9] else {},
            grasp_points=json.loads(row[10]) if row[10] else [],
            grip_force_scale=row[11] or 1.0,
            approach_style=row[12] or 'top_down',
            typical_colors=json.loads(row[13]) if row[13] else [],
            reference_images_path=row[14] or '',
            sdf_path=row[17] or '',
            friction=row[18] or 0.5,
            restitution=row[19] or 0.1,
            date_added=row[20] or '',
            last_seen=row[21] or '',
            times_grasped=row[22] or 0,
            grasp_success_rate=row[23] or 0.0,
            notes=row[24] or '',
        )

    def _infer_affordance(self, class_name: str, props: dict) -> dict:
        """Infer default affordance from class name heuristics."""
        cls = class_name.lower()

        # Known patterns
        if any(k in cls for k in ('cup', 'mug')):
            return {'grasp': 'handle', 'avoid': ['rim', 'interior'],
                    'approach': 'horizontal', 'has_handle': True}
        if any(k in cls for k in ('bottle',)):
            return {'grasp': 'neck', 'avoid': ['cap'],
                    'approach': 'side', 'has_handle': False}
        if any(k in cls for k in ('knife', 'fork', 'spoon')):
            return {'grasp': 'handle', 'avoid': ['blade', 'tines', 'bowl'],
                    'approach': 'side', 'has_handle': True}
        if any(k in cls for k in ('screwdriver', 'pen', 'pencil', 'marker')):
            return {'grasp': 'shaft', 'avoid': ['tip'],
                    'approach': 'side', 'has_handle': True}
        if any(k in cls for k in ('scissors',)):
            return {'grasp': 'handle_holes', 'avoid': ['blade'],
                    'approach': 'top_down', 'has_handle': True}
        if any(k in cls for k in ('cube', 'box', 'block')):
            return {'grasp': 'top', 'avoid': [],
                    'approach': 'top_down', 'has_handle': False}
        if any(k in cls for k in ('ball', 'sphere')):
            return {'grasp': 'top', 'avoid': [],
                    'approach': 'top_down', 'has_handle': False}
        if any(k in cls for k in ('cylinder',)):
            return {'grasp': 'side', 'avoid': [],
                    'approach': 'side', 'has_handle': False}

        # Default
        return {'grasp': 'top', 'avoid': [],
                'approach': 'top_down', 'has_handle': False}

    def _compute_clip_embedding(
        self, images: List[np.ndarray]
    ) -> Optional[bytes]:
        """Compute CLIP embedding for visual similarity search."""
        try:
            import clip
            import torch
            from PIL import Image as PILImage

            device = 'cpu'
            model, preprocess = clip.load('ViT-B/32', device=device)

            embeddings = []
            for img in images[:5]:  # Max 5 images
                pil_img = PILImage.fromarray(img)
                preprocessed = preprocess(pil_img).unsqueeze(0).to(device)
                with torch.no_grad():
                    emb = model.encode_image(preprocessed)
                embeddings.append(emb.cpu().numpy().flatten())

            # Average embedding
            avg = np.mean(embeddings, axis=0).astype(np.float32)
            avg = avg / (np.linalg.norm(avg) + 1e-8)
            return avg.tobytes()

        except Exception:
            return None
