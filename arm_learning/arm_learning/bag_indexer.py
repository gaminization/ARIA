#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Bag Indexer
SQLite database indexing all recorded bags for search.

Usage:
  # As library
  indexer = BagIndexer()
  indexer.index_directory('~/aria_bags')
  results = indexer.search(object_class='cup', failure_only=True)

  # As CLI
  python3 bag_indexer.py index ~/aria_bags
  python3 bag_indexer.py search --object cup --failure-only
  python3 bag_indexer.py stats
═══════════════════════════════════════════════════════════════
"""
import argparse
import json
import os
import sqlite3
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from glob import glob
from typing import Dict, List, Optional

import yaml


# ═══════════════════════════════════════════════════════════════
# Data types
# ═══════════════════════════════════════════════════════════════
@dataclass
class BagInfo:
    id: int = 0
    filepath: str = ''
    start_time: str = ''
    duration_s: float = 0.0
    mode: str = 'sim'
    recording_mode: str = 'standard'
    task_count: int = 0
    failure_count: int = 0
    topics: List[str] = field(default_factory=list)
    objects: List[str] = field(default_factory=list)
    size_mb: float = 0.0
    metadata: dict = field(default_factory=dict)


# ═══════════════════════════════════════════════════════════════
# Database schema
# ═══════════════════════════════════════════════════════════════
CREATE_TABLE_SQL = """
CREATE TABLE IF NOT EXISTS bag_files (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    filepath TEXT UNIQUE NOT NULL,
    start_time TEXT,
    end_time TEXT,
    duration_s REAL DEFAULT 0,
    mode TEXT DEFAULT 'sim',
    recording_mode TEXT DEFAULT 'standard',
    task_count INTEGER DEFAULT 0,
    failure_count INTEGER DEFAULT 0,
    topics TEXT DEFAULT '[]',
    objects TEXT DEFAULT '[]',
    size_bytes INTEGER DEFAULT 0,
    system_version TEXT DEFAULT '',
    metadata_json TEXT DEFAULT '{}',
    indexed_at TEXT
)
"""

CREATE_INDEX_SQL = [
    "CREATE INDEX IF NOT EXISTS idx_start_time ON bag_files(start_time)",
    "CREATE INDEX IF NOT EXISTS idx_mode ON bag_files(mode)",
    "CREATE INDEX IF NOT EXISTS idx_recording_mode ON bag_files(recording_mode)",
]


# ═══════════════════════════════════════════════════════════════
# BagIndexer
# ═══════════════════════════════════════════════════════════════
class BagIndexer:
    """Index and search recorded ROS2 bags."""

    def __init__(self, db_path: str = ''):
        if not db_path:
            db_path = os.path.expanduser(
                '~/aria_bags/bag_index.db')
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

    # ── Indexing ───────────────────────────────────────────
    def index_directory(self, bag_dir: str = '') -> int:
        """
        Scan bag directory and index all bags with metadata.
        Returns count of newly indexed bags.
        """
        if not bag_dir:
            bag_dir = os.path.expanduser('~/aria_bags')

        count = 0

        # Find all metadata YAML files
        for root, dirs, files in os.walk(bag_dir):
            for f in files:
                if f.endswith('_metadata.yaml'):
                    meta_path = os.path.join(root, f)
                    bag_path = meta_path.replace('_metadata.yaml', '')

                    if self._is_indexed(bag_path):
                        continue

                    try:
                        meta = self._load_metadata(meta_path)
                        self._insert_bag(bag_path, meta)
                        count += 1
                    except Exception as e:
                        print(f"  ⚠ Failed to index {meta_path}: {e}")

        # Also index bags without metadata (from manual recordings)
        for root, dirs, files in os.walk(bag_dir):
            for d in dirs:
                if d.startswith('session_') or d.startswith('failure_'):
                    bag_path = os.path.join(root, d)
                    if not self._is_indexed(bag_path):
                        meta = self._infer_metadata(bag_path)
                        self._insert_bag(bag_path, meta)
                        count += 1

        return count

    def index_bag(self, bag_path: str, metadata: dict = None):
        """Index a single bag file."""
        if metadata is None:
            meta_path = bag_path + '_metadata.yaml'
            if os.path.exists(meta_path):
                metadata = self._load_metadata(meta_path)
            else:
                metadata = self._infer_metadata(bag_path)
        self._insert_bag(bag_path, metadata)

    def _is_indexed(self, filepath: str) -> bool:
        with self._conn() as conn:
            row = conn.execute(
                "SELECT id FROM bag_files WHERE filepath = ?",
                (filepath,)).fetchone()
            return row is not None

    def _load_metadata(self, meta_path: str) -> dict:
        with open(meta_path, 'r') as f:
            return yaml.safe_load(f) or {}

    def _infer_metadata(self, bag_path: str) -> dict:
        """Infer metadata for bags without a YAML sidecar."""
        name = os.path.basename(bag_path)
        meta = {
            'bag_filepath': bag_path,
            'recording_mode': 'standard',
        }

        # Infer from name pattern: session_HHMMSS_standard
        parts = name.split('_')
        if len(parts) >= 3:
            meta['recording_mode'] = parts[-1] if parts[-1] in (
                'standard', 'full', 'investigation') else 'standard'

        if name.startswith('failure_'):
            meta['recording_mode'] = 'investigation'
            meta['failure_events'] = [{'type': 'unknown'}]

        # Get size
        if os.path.isdir(bag_path):
            total = sum(
                os.path.getsize(os.path.join(dp, f))
                for dp, dn, fnames in os.walk(bag_path)
                for f in fnames)
            meta['size_bytes'] = total

        # Infer date from parent dir
        parent = os.path.basename(os.path.dirname(bag_path))
        meta['start_time'] = parent  # e.g., "2026-07-01"

        return meta

    def _insert_bag(self, filepath: str, meta: dict):
        with self._conn() as conn:
            conn.execute("""
                INSERT OR REPLACE INTO bag_files
                (filepath, start_time, end_time, duration_s, mode,
                 recording_mode, task_count, failure_count,
                 topics, objects, size_bytes, system_version,
                 metadata_json, indexed_at)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """, (
                filepath,
                meta.get('start_time', ''),
                meta.get('end_time', ''),
                meta.get('duration_s', 0),
                meta.get('mode', 'sim'),
                meta.get('recording_mode', 'standard'),
                meta.get('tasks_attempted', 0),
                len(meta.get('failure_events', [])),
                json.dumps(meta.get('topics_recorded', [])),
                json.dumps(meta.get('objects_in_scene', [])),
                meta.get('size_bytes', 0),
                meta.get('system_version', ''),
                json.dumps(meta),
                datetime.now().isoformat(),
            ))
            conn.commit()

    # ── Search ─────────────────────────────────────────────
    def search(
        self,
        object_class: str = '',
        failure_type: str = '',
        date_from: str = '',
        date_to: str = '',
        success_only: bool = False,
        failure_only: bool = False,
        recording_mode: str = '',
        limit: int = 50,
    ) -> List[BagInfo]:
        """Search indexed bags."""
        conditions = []
        params = []

        if object_class:
            conditions.append("objects LIKE ?")
            params.append(f'%"{object_class}"%')

        if failure_only:
            conditions.append("failure_count > 0")

        if success_only:
            conditions.append("failure_count = 0")
            conditions.append("task_count > 0")

        if date_from:
            conditions.append("start_time >= ?")
            params.append(date_from)

        if date_to:
            conditions.append("start_time <= ?")
            params.append(date_to)

        if recording_mode:
            conditions.append("recording_mode = ?")
            params.append(recording_mode)

        if failure_type:
            conditions.append("metadata_json LIKE ?")
            params.append(f'%{failure_type}%')

        where = " AND ".join(conditions) if conditions else "1=1"
        query = f"""
            SELECT id, filepath, start_time, duration_s, mode,
                   recording_mode, task_count, failure_count,
                   topics, objects, size_bytes, metadata_json
            FROM bag_files
            WHERE {where}
            ORDER BY start_time DESC
            LIMIT ?
        """
        params.append(limit)

        results = []
        with self._conn() as conn:
            for row in conn.execute(query, params):
                info = BagInfo(
                    id=row[0],
                    filepath=row[1],
                    start_time=row[2],
                    duration_s=row[3],
                    mode=row[4],
                    recording_mode=row[5],
                    task_count=row[6],
                    failure_count=row[7],
                    topics=json.loads(row[8]) if row[8] else [],
                    objects=json.loads(row[9]) if row[9] else [],
                    size_mb=row[10] / 1024 / 1024,
                    metadata=json.loads(row[11]) if row[11] else {},
                )
                results.append(info)

        return results

    # ── Statistics ─────────────────────────────────────────
    def get_stats(self) -> dict:
        """Get summary statistics of all indexed bags."""
        with self._conn() as conn:
            row = conn.execute("""
                SELECT
                    COUNT(*),
                    COALESCE(SUM(duration_s), 0),
                    COALESCE(SUM(size_bytes), 0),
                    COALESCE(SUM(task_count), 0),
                    COALESCE(SUM(failure_count), 0)
                FROM bag_files
            """).fetchone()

            return {
                'total_bags': row[0],
                'total_duration_hours': round(row[1] / 3600, 1),
                'total_size_gb': round(row[2] / 1024**3, 2),
                'total_tasks': row[3],
                'total_failures': row[4],
                'success_rate': (
                    round((row[3] - row[4]) / row[3] * 100, 1)
                    if row[3] > 0 else 0.0),
            }


# ═══════════════════════════════════════════════════════════════
# CLI
# ═══════════════════════════════════════════════════════════════
def main():
    parser = argparse.ArgumentParser(
        prog='aria-bag-indexer',
        description='ARIA Bag Indexer — Index and search recorded ROS2 bags')
    sub = parser.add_subparsers(dest='command')

    # index
    idx = sub.add_parser('index', help='Index bags in directory')
    idx.add_argument('directory', nargs='?',
                     default=os.path.expanduser('~/aria_bags'))

    # search
    srch = sub.add_parser('search', help='Search indexed bags')
    srch.add_argument('--object', default='', help='Filter by object class')
    srch.add_argument('--failure', default='', help='Filter by failure type')
    srch.add_argument('--date-from', default='', help='Start date')
    srch.add_argument('--date-to', default='', help='End date')
    srch.add_argument('--failure-only', action='store_true')
    srch.add_argument('--success-only', action='store_true')
    srch.add_argument('--limit', type=int, default=20)

    # stats
    sub.add_parser('stats', help='Show bag statistics')

    args = parser.parse_args()
    indexer = BagIndexer()

    if args.command == 'index':
        count = indexer.index_directory(args.directory)
        print(f"✅ Indexed {count} new bag(s)")

    elif args.command == 'search':
        results = indexer.search(
            object_class=args.object,
            failure_type=args.failure,
            date_from=args.date_from,
            date_to=args.date_to,
            failure_only=args.failure_only,
            success_only=args.success_only,
            limit=args.limit,
        )
        if not results:
            print("No matching bags found")
        else:
            print(f"\n{'ID':>4} {'Date':>12} {'Mode':>8} "
                  f"{'Tasks':>6} {'Fails':>6} {'Size':>8} Path")
            print("─" * 80)
            for r in results:
                date = r.start_time[:10] if r.start_time else '?'
                print(f"{r.id:4d} {date:>12} {r.recording_mode:>8} "
                      f"{r.task_count:6d} {r.failure_count:6d} "
                      f"{r.size_mb:7.1f}M {os.path.basename(r.filepath)}")

    elif args.command == 'stats':
        stats = indexer.get_stats()
        print("\n═══════════════════════════════════════════")
        print("  ARIA Bag Statistics")
        print("═══════════════════════════════════════════")
        print(f"  Total bags:     {stats['total_bags']}")
        print(f"  Total duration: {stats['total_duration_hours']} hours")
        print(f"  Total size:     {stats['total_size_gb']} GB")
        print(f"  Total tasks:    {stats['total_tasks']}")
        print(f"  Total failures: {stats['total_failures']}")
        print(f"  Success rate:   {stats['success_rate']}%")
        print("═══════════════════════════════════════════\n")

    else:
        parser.print_help()


if __name__ == '__main__':
    main()
