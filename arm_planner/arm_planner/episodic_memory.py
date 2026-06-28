#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Episodic Memory — Cross-Session Task Recall
Stores complete task episodes for recall and plan reuse.

Enables:
  "do what we did yesterday"
  "repeat last task"
  "do the same but with the blue cube"

Uses SQLite for persistence + sentence-transformers embeddings
for semantic search. Embeddings run on CPU — no VRAM used.

Database: arm_planner/data/episodes.db
═══════════════════════════════════════════════════════════════
"""
import json
import logging
import os
import sqlite3
import time
import uuid
from datetime import datetime, timedelta
from typing import Dict, List, Optional

import numpy as np

logger = logging.getLogger(__name__)

# ═══════════════════════════════════════════════════════════════
# Embedding Model (lazy loaded — CPU only)
# ═══════════════════════════════════════════════════════════════
_EMBEDDING_MODEL = None
_EMBEDDING_DIM = 384  # all-MiniLM-L6-v2 output dimension


def _get_embedding_model():
    """Lazy-load the sentence-transformers embedding model."""
    global _EMBEDDING_MODEL
    if _EMBEDDING_MODEL is None:
        try:
            from sentence_transformers import SentenceTransformer
            _EMBEDDING_MODEL = SentenceTransformer(
                'all-MiniLM-L6-v2',
                device='cpu',  # Never use GPU for embeddings
            )
            logger.info("Loaded sentence-transformers embedding model (CPU)")
        except ImportError:
            logger.warning(
                "sentence-transformers not installed. "
                "Episodic memory semantic search will be disabled. "
                "Install with: pip install sentence-transformers")
            _EMBEDDING_MODEL = False  # Mark as attempted but unavailable
    return _EMBEDDING_MODEL if _EMBEDDING_MODEL is not False else None


def _compute_embedding(text: str) -> Optional[np.ndarray]:
    """Compute a 384-dim embedding for a text string."""
    model = _get_embedding_model()
    if model is None:
        return None
    try:
        embedding = model.encode(text, convert_to_numpy=True)
        return embedding.astype(np.float32)
    except Exception as e:
        logger.error(f"Embedding computation failed: {e}")
        return None


def _cosine_similarity(a: np.ndarray, b: np.ndarray) -> float:
    """Compute cosine similarity between two vectors."""
    norm_a = np.linalg.norm(a)
    norm_b = np.linalg.norm(b)
    if norm_a == 0 or norm_b == 0:
        return 0.0
    return float(np.dot(a, b) / (norm_a * norm_b))


# ═══════════════════════════════════════════════════════════════
# Episode Data Class
# ═══════════════════════════════════════════════════════════════
class Episode:
    """A single recorded task episode."""

    def __init__(
        self,
        id: int = 0,
        command: str = "",
        goal: str = "",
        plan_json: str = "",
        objects_involved: Optional[List[str]] = None,
        success: bool = False,
        duration_s: float = 0.0,
        timestamp: str = "",
        session_id: str = "",
        embedding: Optional[np.ndarray] = None,
    ):
        self.id = id
        self.command = command
        self.goal = goal
        self.plan_json = plan_json
        self.objects_involved = objects_involved or []
        self.success = success
        self.duration_s = duration_s
        self.timestamp = timestamp or datetime.now().isoformat()
        self.session_id = session_id
        self.embedding = embedding

    @property
    def plan(self) -> Optional[dict]:
        """Parse the stored plan JSON."""
        if not self.plan_json:
            return None
        try:
            return json.loads(self.plan_json)
        except json.JSONDecodeError:
            return None

    @property
    def age_hours(self) -> float:
        """Hours since this episode was recorded."""
        try:
            dt = datetime.fromisoformat(self.timestamp)
            delta = datetime.now() - dt
            return delta.total_seconds() / 3600
        except (ValueError, TypeError):
            return float('inf')

    @property
    def age_description(self) -> str:
        """Human-readable age description."""
        hours = self.age_hours
        if hours < 1:
            return "just now"
        elif hours < 24:
            return f"{hours:.0f} hours ago"
        elif hours < 48:
            return "yesterday"
        else:
            days = hours / 24
            return f"{days:.0f} days ago"

    def summary(self) -> str:
        """One-line summary of the episode."""
        outcome = "✅" if self.success else "❌"
        return (
            f"{outcome} \"{self.command}\" → {self.goal} "
            f"({self.age_description}, {self.duration_s:.1f}s)")

    def __repr__(self) -> str:
        return f"Episode(id={self.id}, command={self.command!r})"


# ═══════════════════════════════════════════════════════════════
# Episodic Memory
# ═══════════════════════════════════════════════════════════════
class EpisodicMemory:
    """
    Persistent episodic memory for cross-session task recall.

    Features:
      - SQLite-backed episode storage
      - Semantic search via sentence-transformers embeddings
      - Temporal reference resolution
      - Planning context injection for LLM
    """

    def __init__(self, db_path: str = ""):
        if not db_path:
            db_path = os.path.join(
                os.path.dirname(os.path.dirname(
                    os.path.abspath(__file__))),
                'data', 'episodes.db')
        self.db_path = db_path
        self.session_id = uuid.uuid4().hex[:12]
        self._init_db()

    # ── Database initialization ────────────────────────────
    def _init_db(self):
        """Create the episodes table if it doesn't exist."""
        os.makedirs(os.path.dirname(self.db_path), exist_ok=True)
        conn = sqlite3.connect(self.db_path)
        conn.execute('''CREATE TABLE IF NOT EXISTS episodes (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            command TEXT NOT NULL,
            goal TEXT DEFAULT '',
            plan_json TEXT DEFAULT '',
            objects_involved TEXT DEFAULT '[]',
            success INTEGER DEFAULT 0,
            duration_s REAL DEFAULT 0.0,
            timestamp TEXT NOT NULL,
            session_id TEXT DEFAULT '',
            command_embedding BLOB
        )''')
        conn.execute('''CREATE INDEX IF NOT EXISTS idx_timestamp
            ON episodes(timestamp)''')
        conn.execute('''CREATE INDEX IF NOT EXISTS idx_session
            ON episodes(session_id)''')
        conn.commit()
        conn.close()
        logger.info(f"Episodic memory initialized at {self.db_path}")

    # ── Record episodes ────────────────────────────────────
    def record_episode(
        self,
        command: str,
        goal: str = "",
        plan_json: str = "",
        objects_involved: Optional[List[str]] = None,
        success: bool = False,
        duration_s: float = 0.0,
    ) -> int:
        """
        Record a completed task episode.

        Returns the episode ID.
        """
        embedding = _compute_embedding(command)
        embedding_blob = (
            embedding.tobytes() if embedding is not None else None)

        objects_json = json.dumps(objects_involved or [])
        timestamp = datetime.now().isoformat()

        conn = sqlite3.connect(self.db_path)
        cursor = conn.execute(
            '''INSERT INTO episodes
            (command, goal, plan_json, objects_involved, success,
             duration_s, timestamp, session_id, command_embedding)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)''',
            (command, goal, plan_json, objects_json,
             int(success), duration_s, timestamp,
             self.session_id, embedding_blob))
        episode_id = cursor.lastrowid
        conn.commit()
        conn.close()

        logger.info(
            f"Recorded episode {episode_id}: "
            f"\"{command}\" ({'success' if success else 'failed'})")
        return episode_id

    # ── Query episodes ─────────────────────────────────────
    def get_last_episode(self) -> Optional[Episode]:
        """Get the most recent episode."""
        conn = sqlite3.connect(self.db_path)
        row = conn.execute(
            'SELECT * FROM episodes ORDER BY id DESC LIMIT 1'
        ).fetchone()
        conn.close()
        return self._row_to_episode(row) if row else None

    def get_episodes_by_date(
        self,
        date_str: str,  # YYYY-MM-DD
    ) -> List[Episode]:
        """Get all episodes from a specific date."""
        conn = sqlite3.connect(self.db_path)
        rows = conn.execute(
            "SELECT * FROM episodes WHERE timestamp LIKE ? ORDER BY id",
            (f"{date_str}%",)
        ).fetchall()
        conn.close()
        return [self._row_to_episode(r) for r in rows]

    def get_recent_episodes(self, n: int = 10) -> List[Episode]:
        """Get the N most recent episodes."""
        conn = sqlite3.connect(self.db_path)
        rows = conn.execute(
            'SELECT * FROM episodes ORDER BY id DESC LIMIT ?',
            (n,)
        ).fetchall()
        conn.close()
        return [self._row_to_episode(r) for r in rows]

    def get_successful_episodes(self, n: int = 10) -> List[Episode]:
        """Get recent successful episodes."""
        conn = sqlite3.connect(self.db_path)
        rows = conn.execute(
            'SELECT * FROM episodes WHERE success=1 '
            'ORDER BY id DESC LIMIT ?',
            (n,)
        ).fetchall()
        conn.close()
        return [self._row_to_episode(r) for r in rows]

    def count_episodes(self) -> int:
        """Total number of recorded episodes."""
        conn = sqlite3.connect(self.db_path)
        count = conn.execute(
            'SELECT COUNT(*) FROM episodes').fetchone()[0]
        conn.close()
        return count

    # ── Semantic search ────────────────────────────────────
    def find_similar_tasks(
        self,
        query: str,
        top_k: int = 3,
    ) -> List[Episode]:
        """
        Find tasks semantically similar to a query string.

        Uses cosine similarity between sentence-transformers
        embeddings. Falls back to keyword matching if
        embeddings are unavailable.
        """
        query_embedding = _compute_embedding(query)

        if query_embedding is None:
            # Fallback: keyword matching
            return self._keyword_search(query, top_k)

        # Load all episodes with embeddings
        conn = sqlite3.connect(self.db_path)
        rows = conn.execute(
            'SELECT * FROM episodes WHERE command_embedding IS NOT NULL'
        ).fetchall()
        conn.close()

        scored_episodes = []
        for row in rows:
            episode = self._row_to_episode(row)
            if episode.embedding is not None:
                similarity = _cosine_similarity(
                    query_embedding, episode.embedding)
                # Slight recency boost (newer = better)
                recency_boost = max(0, 0.05 * (1.0 - min(
                    episode.age_hours / 720, 1.0)))  # Decays over 30 days
                score = similarity + recency_boost
                scored_episodes.append((score, episode))

        # Sort by score descending
        scored_episodes.sort(key=lambda x: x[0], reverse=True)
        return [ep for _, ep in scored_episodes[:top_k]]

    def _keyword_search(
        self,
        query: str,
        top_k: int,
    ) -> List[Episode]:
        """Fallback keyword search when embeddings unavailable."""
        words = set(query.lower().split())
        conn = sqlite3.connect(self.db_path)
        rows = conn.execute(
            'SELECT * FROM episodes ORDER BY id DESC LIMIT 100'
        ).fetchall()
        conn.close()

        scored = []
        for row in rows:
            episode = self._row_to_episode(row)
            ep_words = set(episode.command.lower().split())
            overlap = len(words & ep_words)
            if overlap > 0:
                scored.append((overlap, episode))

        scored.sort(key=lambda x: x[0], reverse=True)
        return [ep for _, ep in scored[:top_k]]

    # ── Temporal reference resolution ──────────────────────
    def resolve_temporal_reference(
        self,
        reference: str,
    ) -> Optional[Episode]:
        """
        Resolve temporal references to specific episodes.

        Handles:
          "last task"         → most recent episode
          "yesterday's task"  → episodes from yesterday, most recent
          "what we did before"→ semantic search + recency weighting
          "the sorting task"  → semantic search for "sort"
          "do it again"       → most recent successful episode
        """
        ref_lower = reference.lower().strip()

        # "last task" / "previous task" / "again"
        if any(w in ref_lower for w in ['last', 'previous', 'again', 'repeat']):
            return self.get_last_episode()

        # "yesterday"
        if 'yesterday' in ref_lower:
            yesterday = (datetime.now() - timedelta(days=1))
            episodes = self.get_episodes_by_date(
                yesterday.strftime('%Y-%m-%d'))
            return episodes[-1] if episodes else None

        # "today"
        if 'today' in ref_lower:
            today = datetime.now().strftime('%Y-%m-%d')
            episodes = self.get_episodes_by_date(today)
            return episodes[-1] if episodes else None

        # "this morning" / "this afternoon" / "earlier"
        if any(w in ref_lower for w in ['earlier', 'morning', 'afternoon']):
            today = datetime.now().strftime('%Y-%m-%d')
            episodes = self.get_episodes_by_date(today)
            return episodes[0] if episodes else None

        # Semantic search for specific task references
        # e.g., "the sorting task", "when we stacked blocks"
        similar = self.find_similar_tasks(reference, top_k=1)
        return similar[0] if similar else None

    # ── Planning context injection ─────────────────────────
    def get_planning_context(
        self,
        command: str,
        top_k: int = 3,
    ) -> str:
        """
        Build a context string for LLM planning based on
        similar past tasks.

        Injected into the system prompt at planning time.
        """
        similar = self.find_similar_tasks(command, top_k=top_k)

        if not similar:
            return "No relevant past tasks found."

        lines = ["Relevant past tasks:"]
        for i, ep in enumerate(similar):
            outcome = "succeeded" if ep.success else "failed"
            plan = ep.plan
            plan_summary = ""
            if plan:
                actions = plan.get('actions', [])
                if actions:
                    steps = [a.get('skill') or a.get('agent', '?')
                             for a in actions[:5]]
                    plan_summary = f"Steps: {' → '.join(steps)}"

            objects_str = ""
            if ep.objects_involved:
                objects_str = f"Objects: {', '.join(ep.objects_involved)}"

            lines.append(
                f"  {i+1}. \"{ep.command}\" → {outcome} "
                f"({ep.age_description})")
            if plan_summary:
                lines.append(f"     {plan_summary}")
            if objects_str:
                lines.append(f"     {objects_str}")

        return '\n'.join(lines)

    # ── Statistics ─────────────────────────────────────────
    def get_stats(self) -> Dict:
        """Get episodic memory statistics."""
        conn = sqlite3.connect(self.db_path)
        total = conn.execute(
            'SELECT COUNT(*) FROM episodes').fetchone()[0]
        successes = conn.execute(
            'SELECT COUNT(*) FROM episodes WHERE success=1'
        ).fetchone()[0]
        sessions = conn.execute(
            'SELECT COUNT(DISTINCT session_id) FROM episodes'
        ).fetchone()[0]
        conn.close()

        return {
            'total_episodes': total,
            'successful': successes,
            'failed': total - successes,
            'success_rate': successes / max(total, 1),
            'sessions': sessions,
            'current_session': self.session_id,
        }

    # ── Internal helpers ───────────────────────────────────
    def _row_to_episode(self, row) -> Episode:
        """Convert a database row to an Episode object."""
        embedding = None
        if row[9]:  # command_embedding BLOB
            try:
                embedding = np.frombuffer(
                    row[9], dtype=np.float32).copy()
            except (ValueError, TypeError):
                pass

        objects = []
        try:
            objects = json.loads(row[4]) if row[4] else []
        except json.JSONDecodeError:
            pass

        return Episode(
            id=row[0],
            command=row[1],
            goal=row[2],
            plan_json=row[3],
            objects_involved=objects,
            success=bool(row[5]),
            duration_s=row[6],
            timestamp=row[7],
            session_id=row[8],
            embedding=embedding,
        )
