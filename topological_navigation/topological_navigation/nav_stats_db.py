"""SQLite persistence module for topological navigation traversal statistics.

This module provides the ``NavStatsDB`` class which handles all database
operations for recording and querying edge traversal statistics.  It is
designed to be used by the navigation server and is intentionally kept
independent of any ROS 2 imports so that it can also be used from
standalone analysis scripts.

Schema
------
``traversals`` table stores one row per traversal attempt:

    id              – auto-increment primary key
    map_name        – topological map pointset name
    map_hash        – SHA-1 of the serialised map YAML (version fingerprint)
    edge_id         – edge identifier (e.g. ``nodeA_nodeB``)
    origin          – source node name
    target          – destination node name
    status          – ``'success'``, ``'failed'``, or ``'aborted'``
    failure_reason  – ``'none'``, ``'cancelled'``, ``'lookup_failed'``, etc.
    start_time      – ISO-8601 UTC timestamp
    end_time        – ISO-8601 UTC timestamp
    duration_s      – traversal duration in seconds
    avg_speed       – edge_length / duration_s (m/s); NULL if unknown
    is_segment      – 1 if this traversal was part of a multi-edge segment
    segment_edges   – JSON array of all edge IDs in the segment (or NULL)
    created_at      – row insertion timestamp (UTC)
"""

import hashlib
import json
import sqlite3
from datetime import datetime, timezone
from pathlib import Path
from typing import List, Optional


# ---------------------------------------------------------------------------
# Schema
# ---------------------------------------------------------------------------

_CREATE_TRAVERSALS = """
CREATE TABLE IF NOT EXISTS traversals (
    id             INTEGER PRIMARY KEY AUTOINCREMENT,
    map_name       TEXT    NOT NULL DEFAULT '',
    map_hash       TEXT    NOT NULL DEFAULT '',
    edge_id        TEXT    NOT NULL DEFAULT '',
    origin         TEXT    NOT NULL DEFAULT '',
    target         TEXT    NOT NULL DEFAULT '',
    status         TEXT    NOT NULL DEFAULT 'unknown',
    failure_reason TEXT    NOT NULL DEFAULT 'none',
    start_time     TEXT,
    end_time       TEXT,
    duration_s     REAL,
    avg_speed      REAL,
    is_segment     INTEGER NOT NULL DEFAULT 0,
    segment_edges  TEXT,
    created_at     TEXT    NOT NULL DEFAULT (datetime('now'))
);
"""

_CREATE_IDX_EDGE = """
CREATE INDEX IF NOT EXISTS idx_traversals_edge_id
    ON traversals (edge_id);
"""

_CREATE_IDX_MAP = """
CREATE INDEX IF NOT EXISTS idx_traversals_map_name
    ON traversals (map_name);
"""

_INSERT_TRAVERSAL = """
INSERT INTO traversals (
    map_name, map_hash, edge_id, origin, target,
    status, failure_reason,
    start_time, end_time, duration_s, avg_speed,
    is_segment, segment_edges
) VALUES (
    :map_name, :map_hash, :edge_id, :origin, :target,
    :status, :failure_reason,
    :start_time, :end_time, :duration_s, :avg_speed,
    :is_segment, :segment_edges
);
"""


# ---------------------------------------------------------------------------
# Public helpers
# ---------------------------------------------------------------------------

def compute_map_hash(map_yaml_str: str) -> str:
    """Return a short SHA-1 hex digest of the serialised map YAML string."""
    return hashlib.sha1(map_yaml_str.encode("utf-8", errors="replace")).hexdigest()[:12]


def _iso(dt: Optional[datetime]) -> Optional[str]:
    """Convert *dt* to an ISO-8601 UTC string, or return ``None``."""
    if dt is None:
        return None
    if dt.tzinfo is None:
        # Assume local time; store as-is with 'Z' suffix omitted
        return dt.isoformat(timespec="seconds")
    return dt.astimezone(timezone.utc).isoformat(timespec="seconds")


# ---------------------------------------------------------------------------
# Main class
# ---------------------------------------------------------------------------

class NavStatsDB:
    """Thin wrapper around an SQLite database for navigation statistics.

    Parameters
    ----------
    db_path:
        Path to the SQLite file.  The file (and any parent directories)
        are created automatically if they do not exist.
    """

    def __init__(self, db_path: str) -> None:
        path = Path(db_path).expanduser().resolve()
        path.parent.mkdir(parents=True, exist_ok=True)
        self._path = str(path)
        self._conn: Optional[sqlite3.Connection] = None
        self._open()
        self._create_schema()

    # ------------------------------------------------------------------
    # Connection management
    # ------------------------------------------------------------------

    def _open(self) -> None:
        """Open (or re-open) the SQLite connection with WAL mode."""
        self._conn = sqlite3.connect(
            self._path,
            check_same_thread=False,
            timeout=10.0,
        )
        self._conn.execute("PRAGMA journal_mode=WAL;")
        self._conn.execute("PRAGMA foreign_keys=ON;")

    def _create_schema(self) -> None:
        """Create tables and indexes if they do not yet exist."""
        with self._conn:
            self._conn.execute(_CREATE_TRAVERSALS)
            self._conn.execute(_CREATE_IDX_EDGE)
            self._conn.execute(_CREATE_IDX_MAP)

    def close(self) -> None:
        """Close the database connection."""
        if self._conn is not None:
            self._conn.close()
            self._conn = None

    # ------------------------------------------------------------------
    # Write API
    # ------------------------------------------------------------------

    def record_traversal(
        self,
        *,
        map_name: str,
        map_hash: str,
        edge_id: str,
        origin: str,
        target: str,
        status: str,
        failure_reason: str = "none",
        start_time: Optional[datetime] = None,
        end_time: Optional[datetime] = None,
        duration_s: Optional[float] = None,
        edge_length: Optional[float] = None,
        is_segment: bool = False,
        segment_edges: Optional[List[str]] = None,
    ) -> int:
        """Insert one traversal record and return the new row id.

        Parameters
        ----------
        map_name:      Topological map pointset name.
        map_hash:      Short hash of the map YAML for version tracking.
        edge_id:       Edge identifier (``'nodeA_nodeB'``).
        origin:        Source node name.
        target:        Destination node name.
        status:        ``'success'``, ``'failed'``, or ``'aborted'``.
        failure_reason: Human-readable failure category.
        start_time:    Traversal start (``datetime`` object).
        end_time:      Traversal end (``datetime`` object).
        duration_s:    Duration in seconds (derived from timestamps if omitted).
        edge_length:   Euclidean distance between nodes (metres).  Used to
                       compute ``avg_speed = edge_length / duration_s``.
        is_segment:    Whether this traversal was part of a multi-edge segment.
        segment_edges: All edge IDs belonging to the enclosing segment.
        """
        # Derive duration from timestamps if not provided
        if duration_s is None and start_time is not None and end_time is not None:
            duration_s = (end_time - start_time).total_seconds()

        # Average speed (m/s)
        avg_speed: Optional[float] = None
        if (
            edge_length is not None
            and duration_s is not None
            and duration_s > 0.0
        ):
            avg_speed = edge_length / duration_s

        row = {
            "map_name": map_name,
            "map_hash": map_hash,
            "edge_id": edge_id,
            "origin": origin,
            "target": target,
            "status": status,
            "failure_reason": failure_reason,
            "start_time": _iso(start_time),
            "end_time": _iso(end_time),
            "duration_s": duration_s,
            "avg_speed": avg_speed,
            "is_segment": 1 if is_segment else 0,
            "segment_edges": json.dumps(segment_edges) if segment_edges else None,
        }

        with self._conn:
            cur = self._conn.execute(_INSERT_TRAVERSAL, row)
        return cur.lastrowid

    # ------------------------------------------------------------------
    # Read API (used by the standalone stats script)
    # ------------------------------------------------------------------

    def query(self, sql: str, params=()):
        """Execute an arbitrary SELECT and return all rows as dicts."""
        cur = self._conn.execute(sql, params)
        cols = [d[0] for d in cur.description]
        return [dict(zip(cols, row)) for row in cur.fetchall()]

    def edge_ids(self, where: str = "") -> List[str]:
        """Return a sorted list of distinct edge IDs, optionally filtered."""
        sql = "SELECT DISTINCT edge_id FROM traversals"
        if where:
            sql += " WHERE " + where
        sql += " ORDER BY edge_id"
        return [r["edge_id"] for r in self.query(sql)]

    def edge_stats(self, edge_id: str, where: str = "") -> dict:
        """Aggregate statistics for *edge_id*.

        Returns a dict with keys:
            edge_id, total, success, failed, aborted,
            avg_duration_s, min_duration_s, max_duration_s,
            avg_speed, last_traversal
        """
        base = "edge_id = :eid"
        if where:
            base += " AND (" + where + ")"

        sql = """
            SELECT
                edge_id,
                COUNT(*)                                    AS total,
                SUM(CASE WHEN status='success' THEN 1 ELSE 0 END) AS success,
                SUM(CASE WHEN status='failed'  THEN 1 ELSE 0 END) AS failed,
                SUM(CASE WHEN status='aborted' THEN 1 ELSE 0 END) AS aborted,
                AVG(CASE WHEN duration_s IS NOT NULL THEN duration_s END)
                                                            AS avg_duration_s,
                MIN(CASE WHEN duration_s IS NOT NULL THEN duration_s END)
                                                            AS min_duration_s,
                MAX(CASE WHEN duration_s IS NOT NULL THEN duration_s END)
                                                            AS max_duration_s,
                AVG(avg_speed)                              AS avg_speed,
                MAX(end_time)                               AS last_traversal
            FROM traversals
            WHERE """ + base + """
            GROUP BY edge_id
        """
        rows = self.query(sql, {"eid": edge_id})
        return rows[0] if rows else {}
