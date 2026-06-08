#!/usr/bin/env python3
"""Standalone statistics generator for topological navigation traversal data.

Reads an SQLite database produced by the topological navigation server and
writes a YAML document containing per-edge statistics to stdout.

Usage
-----
::

    python3 topo_stats.py <database.db> [--filter <SQL-WHERE-expression>]

Examples
--------
All edges::

    python3 topo_stats.py /data/nav_stats.db

Only edges whose traversal started after a specific date::

    python3 topo_stats.py /data/nav_stats.db \\
        --filter "start_time > '2024-01-01'"

Only edges in a particular map::

    python3 topo_stats.py /data/nav_stats.db \\
        --filter "map_name = 'my_field_map'"

Only edges with a specific action segment membership::

    python3 topo_stats.py /data/nav_stats.db \\
        --filter "is_segment = 1"
"""

import argparse
import sys

import yaml

from topological_navigation.nav_stats_db import NavStatsDB


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description=(
            "Generate edge statistics from a topological navigation "
            "SQLite database."
        ),
    )
    p.add_argument(
        "database",
        help="Path to the SQLite database file.",
    )
    p.add_argument(
        "--filter",
        metavar="SQL_EXPR",
        default="",
        help=(
            "Optional SQL WHERE expression to restrict which traversal "
            "records are included (applied to the traversals table). "
            "Example: \"map_name = 'my_map' AND status = 'success'\""
        ),
    )
    return p


def _compute_all_edge_stats(db: NavStatsDB, where: str) -> dict:
    """Return a dict mapping edge_id -> statistics dict."""
    edge_ids = db.edge_ids(where=where)
    result = {}
    for eid in edge_ids:
        stats = db.edge_stats(eid, where=where)
        if not stats:
            continue
        result[eid] = {
            "total_traversals": int(stats.get("total") or 0),
            "success": int(stats.get("success") or 0),
            "failed": int(stats.get("failed") or 0),
            "aborted": int(stats.get("aborted") or 0),
            "avg_duration_s": _round(stats.get("avg_duration_s")),
            "min_duration_s": _round(stats.get("min_duration_s")),
            "max_duration_s": _round(stats.get("max_duration_s")),
            "avg_speed_m_s": _round(stats.get("avg_speed")),
            "last_traversal": stats.get("last_traversal"),
        }
    return result


def _round(value, decimals: int = 4):
    """Round a float value, returning None if the input is None."""
    if value is None:
        return None
    try:
        return round(float(value), decimals)
    except (TypeError, ValueError):
        return None


def main():
    parser = _build_parser()
    args = parser.parse_args()

    try:
        db = NavStatsDB(args.database)
    except Exception as exc:
        print("ERROR: Cannot open database '%s': %s" % (args.database, exc),
              file=sys.stderr)
        sys.exit(1)

    try:
        edge_stats = _compute_all_edge_stats(db, args.filter)
    except Exception as exc:
        print("ERROR: Query failed: %s" % exc, file=sys.stderr)
        db.close()
        sys.exit(1)
    finally:
        db.close()

    if not edge_stats:
        print("# No traversal records found matching the given filter.",
              file=sys.stderr)
        print("{}", flush=True)
        return

    print(
        yaml.dump(
            {"edge_statistics": edge_stats},
            default_flow_style=False,
            sort_keys=True,
            allow_unicode=True,
        ),
        end="",
        flush=True,
    )


if __name__ == "__main__":
    main()
