#!/usr/bin/env python3
"""topo_stats – CLI tool for topological navigation statistics.

This tool reads an SQLite database produced by the topological navigation
server and provides a variety of commands for inspecting stored maps and
traversal statistics.

Usage
-----
::

    python3 topo_stats.py <database.db> <command> [sub-command] [options]

Top-level commands
------------------
map
    Operations on the stored topological maps table.
traversals
    Operations on the recorded traversal statistics.

Run ``python3 topo_stats.py --help`` or
``python3 topo_stats.py <command> --help`` for detailed help.

Examples
--------
List all stored maps::

    python3 topo_stats.py /data/nav_stats.db map list

Export a map to YAML::

    python3 topo_stats.py /data/nav_stats.db map export my_field_map

Show map metadata::

    python3 topo_stats.py /data/nav_stats.db map show abc123def456

Calculate map road-network statistics::

    python3 topo_stats.py /data/nav_stats.db map stats my_field_map

Import a map YAML file::

    python3 topo_stats.py /data/nav_stats.db map import /path/to/map.yaml

Delete a stored map::

    python3 topo_stats.py /data/nav_stats.db map rm my_field_map

Show traversal summary for all maps::

    python3 topo_stats.py /data/nav_stats.db traversals summary

Show traversal summary filtered to a time window::

    python3 topo_stats.py /data/nav_stats.db traversals summary \\
        --filter "start_time > '2024-01-01'"

Per-edge statistics for a map::

    python3 topo_stats.py /data/nav_stats.db traversals edge_stats my_field_map

Per-edge statistics filtered to successful traversals only::

    python3 topo_stats.py /data/nav_stats.db traversals edge_stats my_field_map \\
        --filter "status = 'success'"

Whole-map traversal statistics::

    python3 topo_stats.py /data/nav_stats.db traversals map_stats my_field_map

Whole-map statistics for a specific time window with top-N failure report::

    python3 topo_stats.py /data/nav_stats.db traversals map_stats my_field_map \\
        --filter "start_time BETWEEN '2024-01-01' AND '2024-06-01'" \\
        --topn_failures=5 --topn_success=5
"""

import argparse
import math
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional

import yaml

from topological_navigation.nav_stats_db import NavStatsDB
from topological_navigation.networkx_utils import build_graph_from_tmap


# ---------------------------------------------------------------------------
# Formatting helpers
# ---------------------------------------------------------------------------

def _round(value, decimals: int = 4):
    """Round a float value, returning None if the input is None."""
    if value is None:
        return None
    try:
        return round(float(value), decimals)
    except (TypeError, ValueError):
        return None


def _fmt(value, decimals: int = 4, na: str = "N/A") -> str:
    """Format a numeric value for table display."""
    if value is None:
        return na
    try:
        return str(round(float(value), decimals))
    except (TypeError, ValueError):
        return str(value)


def _markdown_table(headers: List[str], rows: List[List[str]]) -> str:
    """Render *rows* as a GitHub-Flavoured Markdown table.

    All columns are left-aligned and padded to their widest value so the
    table is readable in a fixed-width terminal as well as in Markdown
    renderers.
    """
    all_rows = [headers] + rows
    widths = [max(len(str(cell)) for cell in col) for col in zip(*all_rows)]
    sep = "| " + " | ".join("-" * w for w in widths) + " |"

    def _row(cells):
        return "| " + " | ".join(str(c).ljust(w) for c, w in zip(cells, widths)) + " |"

    lines = [_row(headers), sep] + [_row(r) for r in rows]
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Map graph helpers (no ROS, pure Python)
# ---------------------------------------------------------------------------

def _parse_map_yaml(map_data: str) -> Optional[Dict[str, Any]]:
    """Parse map YAML text, returning None on failure."""
    try:
        return yaml.safe_load(map_data) or {}
    except Exception as exc:
        print("ERROR: Cannot parse map YAML: %s" % exc, file=sys.stderr)
        return None


def _build_graph(tmap: Dict[str, Any]):
    """Build a NetworkX graph from a parsed tmap dict (no logger)."""
    return build_graph_from_tmap(tmap)


def _map_road_stats(tmap: Dict[str, Any]) -> Dict[str, Any]:
    """Return basic road-network statistics for a topological map.

    Returns a dict with:
        num_nodes, num_edges, total_length_m, bbox_area_sqm
    """
    graph = _build_graph(tmap)
    if graph is None:
        return {"num_nodes": 0, "num_edges": 0, "total_length_m": 0.0,
                "bbox_area_sqm": 0.0}

    num_nodes = graph.number_of_nodes()
    num_edges = graph.number_of_edges()

    # Total roadmap length = sum of all edge Euclidean lengths
    total_length = 0.0
    for u, v in graph.edges():
        dx = graph.nodes[v].get("x", 0) - graph.nodes[u].get("x", 0)
        dy = graph.nodes[v].get("y", 0) - graph.nodes[u].get("y", 0)
        total_length += math.hypot(dx, dy)

    # Bounding box area
    xs = [graph.nodes[n].get("x", 0) for n in graph.nodes()]
    ys = [graph.nodes[n].get("y", 0) for n in graph.nodes()]
    if len(xs) >= 2:
        bbox_area = (max(xs) - min(xs)) * (max(ys) - min(ys))
    else:
        bbox_area = 0.0

    return {
        "num_nodes": num_nodes,
        "num_edges": num_edges,
        "total_length_m": round(total_length, 3),
        "bbox_area_sqm": round(bbox_area, 3),
    }


# ---------------------------------------------------------------------------
# 'map' sub-commands
# ---------------------------------------------------------------------------

def cmd_map_list(db: NavStatsDB, _args) -> int:
    """List all stored maps as a Markdown table."""
    maps = db.list_maps()
    if not maps:
        print("No maps stored in the database.")
        return 0
    rows = [
        [
            str(m["id"]),
            m["map_name"] or "",
            m["map_hash"],
            _fmt(m.get("latitude"), 8),
            _fmt(m.get("longitude"), 8),
            m.get("added_at") or "",
        ]
        for m in maps
    ]
    print(_markdown_table(
        ["id", "map_name", "map_hash", "latitude", "longitude", "added_at"],
        rows,
    ))
    return 0


def cmd_map_show(db: NavStatsDB, args) -> int:
    """Show metadata for a specific map."""
    m = db.get_map(args.identifier)
    if m is None:
        print("ERROR: Map '%s' not found." % args.identifier, file=sys.stderr)
        return 1
    meta: Dict[str, Any] = {
        "map_name": m["map_name"],
        "map_hash": m["map_hash"],
        "latitude": m.get("latitude"),
        "longitude": m.get("longitude"),
        "added_at": m.get("added_at"),
    }
    print(yaml.dump(meta, default_flow_style=False, sort_keys=True,
                    allow_unicode=True), end="")
    return 0


def cmd_map_export(db: NavStatsDB, args) -> int:
    """Export a stored map to YAML (stdout)."""
    m = db.get_map(args.identifier)
    if m is None:
        print("ERROR: Map '%s' not found." % args.identifier, file=sys.stderr)
        return 1
    print(m["map_data"], end="")
    return 0


def cmd_map_import(db: NavStatsDB, args) -> int:
    """Import a YAML map file into the database."""
    p = Path(args.yaml_file)
    if not p.is_file():
        print("ERROR: File not found: %s" % args.yaml_file, file=sys.stderr)
        return 1
    try:
        map_yaml_str = p.read_text(encoding="utf-8")
    except Exception as exc:
        print("ERROR: Cannot read file '%s': %s" % (args.yaml_file, exc),
              file=sys.stderr)
        return 1
    # Validate YAML
    tmap = _parse_map_yaml(map_yaml_str)
    if tmap is None:
        return 1
    map_hash = db.store_map(map_yaml_str)
    map_name = tmap.get("name") or tmap.get("pointset") or "(unknown)"
    print("Imported map '%s' (hash: %s)." % (map_name, map_hash))
    return 0


def cmd_map_rm(db: NavStatsDB, args) -> int:
    """Delete a stored map."""
    count = db.delete_map(args.identifier)
    if count == 0:
        print("ERROR: Map '%s' not found." % args.identifier, file=sys.stderr)
        return 1
    print("Deleted %d map record(s) for '%s'." % (count, args.identifier))
    return 0


def cmd_map_stats(db: NavStatsDB, args) -> int:
    """Calculate road-network statistics for a stored map."""
    m = db.get_map(args.identifier)
    if m is None:
        print("ERROR: Map '%s' not found." % args.identifier, file=sys.stderr)
        return 1
    tmap = _parse_map_yaml(m["map_data"])
    if tmap is None:
        return 1
    road = _map_road_stats(tmap)
    rows = [
        ["map_name", m["map_name"]],
        ["map_hash", m["map_hash"]],
        ["num_nodes", str(road["num_nodes"])],
        ["num_edges", str(road["num_edges"])],
        ["total_length_m", str(road["total_length_m"])],
        ["bbox_area_sqm", str(road["bbox_area_sqm"])],
    ]
    print(_markdown_table(["metric", "value"], rows))
    return 0


# ---------------------------------------------------------------------------
# 'traversals' sub-commands
# ---------------------------------------------------------------------------

def _resolve_map_identifier(db: NavStatsDB, identifier: str):
    """Return (map_name, map_hash) for *identifier*, or exit on ambiguity."""
    m = db.get_map(identifier)
    if m is None:
        print("ERROR: Map '%s' not found in database." % identifier,
              file=sys.stderr)
        sys.exit(1)
    return m["map_name"], m["map_hash"]


def cmd_traversals_summary(db: NavStatsDB, args) -> int:
    """Summarise traversal counts per map with timing and basic stats."""
    where_clause = ""
    if args.filter:
        where_clause = "WHERE " + args.filter
    sql = """
        SELECT
            map_name,
            map_hash,
            COUNT(*)                                          AS total,
            SUM(CASE WHEN status='success' THEN 1 ELSE 0 END) AS success,
            SUM(CASE WHEN status='failed'  THEN 1 ELSE 0 END) AS failed,
            SUM(CASE WHEN status='aborted' THEN 1 ELSE 0 END) AS aborted,
            MIN(start_time)                                   AS first_traversal,
            MAX(end_time)                                     AS last_traversal,
            AVG(CASE WHEN duration_s IS NOT NULL THEN duration_s END)
                                                              AS avg_duration_s,
            AVG(avg_speed)                                    AS avg_speed_m_s
        FROM traversals
        {where}
        GROUP BY map_name, map_hash
        ORDER BY map_name, map_hash
    """.format(where=where_clause)

    rows_data = db.query(sql)
    if not rows_data:
        print("No traversal records found matching the given filter.")
        return 0

    rows = []
    for r in rows_data:
        total = r.get("total") or 0
        success = r.get("success") or 0
        failed = r.get("failed") or 0
        aborted = r.get("aborted") or 0
        rows.append([
            r.get("map_name") or "",
            r.get("map_hash") or "",
            str(total),
            str(success),
            str(failed),
            str(aborted),
            r.get("first_traversal") or "N/A",
            r.get("last_traversal") or "N/A",
            _fmt(r.get("avg_duration_s"), 2),
            _fmt(r.get("avg_speed_m_s"), 4),
        ])
    print(_markdown_table(
        ["map_name", "map_hash", "total", "success", "failed", "aborted",
         "first_traversal", "last_traversal", "avg_dur_s", "avg_spd_m/s"],
        rows,
    ))
    return 0


def cmd_traversals_edge_stats(db: NavStatsDB, args) -> int:
    """Per-edge statistics for a specific map."""
    map_name, map_hash = _resolve_map_identifier(db, args.map_id)

    # Build WHERE combining map constraint with optional user filter
    base_filter = "map_hash = '%s'" % map_hash
    where = base_filter
    if args.filter:
        where += " AND (" + args.filter + ")"

    edge_ids = db.edge_ids(where=where)
    if not edge_ids:
        print("# No traversal records found for map '%s'." % args.map_id,
              file=sys.stderr)
        print("{}", flush=True)
        return 0

    edge_stats: Dict[str, Any] = {}
    for eid in edge_ids:
        s = db.edge_stats(eid, where=where)
        if not s:
            continue
        edge_stats[eid] = {
            "total_traversals": int(s.get("total") or 0),
            "success": int(s.get("success") or 0),
            "failed": int(s.get("failed") or 0),
            "aborted": int(s.get("aborted") or 0),
            "avg_duration_s": _round(s.get("avg_duration_s")),
            "min_duration_s": _round(s.get("min_duration_s")),
            "max_duration_s": _round(s.get("max_duration_s")),
            "avg_speed_m_s": _round(s.get("avg_speed")),
            "last_traversal": s.get("last_traversal"),
        }

    output: Dict[str, Any] = {
        "map_name": map_name,
        "map_hash": map_hash,
        "edge_statistics": edge_stats,
    }
    print(
        yaml.dump(output, default_flow_style=False, sort_keys=True,
                  allow_unicode=True),
        end="",
        flush=True,
    )
    return 0


def cmd_traversals_map_stats(db: NavStatsDB, args) -> int:
    """Whole-map traversal statistics across all edges."""
    map_name, map_hash = _resolve_map_identifier(db, args.map_id)

    base_filter = "map_hash = '%s'" % map_hash
    where = base_filter
    if args.filter:
        where += " AND (" + args.filter + ")"

    sql = """
        SELECT
            COUNT(*)                                            AS total,
            SUM(CASE WHEN status='success' THEN 1 ELSE 0 END)  AS success,
            SUM(CASE WHEN status='failed'  THEN 1 ELSE 0 END)  AS failed,
            SUM(CASE WHEN status='aborted' THEN 1 ELSE 0 END)  AS aborted,
            SUM(CASE WHEN duration_s IS NOT NULL THEN duration_s ELSE 0 END)
                                                                AS total_time_s,
            AVG(avg_speed)                                      AS avg_speed_m_s,
            MAX(avg_speed)                                      AS max_speed_m_s
        FROM traversals
        WHERE {where}
    """.format(where=where)

    agg = db.query(sql)
    if not agg or agg[0].get("total") is None or agg[0]["total"] == 0:
        print("# No traversal records found for map '%s'." % args.map_id,
              file=sys.stderr)
        print("{}", flush=True)
        return 0

    a = agg[0]
    total = int(a.get("total") or 0)
    success = int(a.get("success") or 0)
    failed = int(a.get("failed") or 0)
    aborted = int(a.get("aborted") or 0)

    def _pct(n):
        return round(100.0 * n / total, 2) if total else 0.0

    # Approximate total distance: sum of avg_speed * duration_s for successful
    total_dist_sql = """
        SELECT SUM(avg_speed * duration_s) AS total_dist_m
        FROM traversals
        WHERE avg_speed IS NOT NULL AND duration_s IS NOT NULL AND {where}
    """.format(where=where)
    dist_rows = db.query(total_dist_sql)
    total_dist_m = (dist_rows[0].get("total_dist_m") or 0.0) if dist_rows else 0.0

    result: Dict[str, Any] = {
        "map_name": map_name,
        "map_hash": map_hash,
        "total_traversals": total,
        "total_distance_m": _round(total_dist_m, 3),
        "total_time_s": _round(a.get("total_time_s"), 3),
        "outcomes": {
            "success": success,
            "success_pct": _pct(success),
            "failed": failed,
            "failed_pct": _pct(failed),
            "aborted": aborted,
            "aborted_pct": _pct(aborted),
        },
        "speed": {
            "avg_m_s": _round(a.get("avg_speed_m_s")),
            "max_m_s": _round(a.get("max_speed_m_s")),
        },
    }

    # Top-N edges by failure count
    topn_failures = getattr(args, "topn_failures", None)
    if topn_failures:
        sql_topn = """
            SELECT edge_id,
                   SUM(CASE WHEN status='failed' THEN 1 ELSE 0 END) AS cnt
            FROM traversals
            WHERE {where}
            GROUP BY edge_id
            ORDER BY cnt DESC
            LIMIT {n}
        """.format(where=where, n=int(topn_failures))
        result["top_edges_by_failures"] = [
            {"edge_id": r["edge_id"], "failed": int(r["cnt"] or 0)}
            for r in db.query(sql_topn)
            if (r.get("cnt") or 0) > 0
        ]

    # Top-N edges by success count
    topn_success = getattr(args, "topn_success", None)
    if topn_success:
        sql_topn = """
            SELECT edge_id,
                   SUM(CASE WHEN status='success' THEN 1 ELSE 0 END) AS cnt
            FROM traversals
            WHERE {where}
            GROUP BY edge_id
            ORDER BY cnt DESC
            LIMIT {n}
        """.format(where=where, n=int(topn_success))
        result["top_edges_by_success"] = [
            {"edge_id": r["edge_id"], "success": int(r["cnt"] or 0)}
            for r in db.query(sql_topn)
            if (r.get("cnt") or 0) > 0
        ]

    # Top-N edges by aborted count
    topn_aborted = getattr(args, "topn_aborted", None)
    if topn_aborted:
        sql_topn = """
            SELECT edge_id,
                   SUM(CASE WHEN status='aborted' THEN 1 ELSE 0 END) AS cnt
            FROM traversals
            WHERE {where}
            GROUP BY edge_id
            ORDER BY cnt DESC
            LIMIT {n}
        """.format(where=where, n=int(topn_aborted))
        result["top_edges_by_aborted"] = [
            {"edge_id": r["edge_id"], "aborted": int(r["cnt"] or 0)}
            for r in db.query(sql_topn)
            if (r.get("cnt") or 0) > 0
        ]

    print(
        yaml.dump(result, default_flow_style=False, sort_keys=True,
                  allow_unicode=True),
        end="",
        flush=True,
    )
    return 0


# ---------------------------------------------------------------------------
# Argument parser
# ---------------------------------------------------------------------------

def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="topo_stats",
        description=(
            "topo_stats – CLI tool for topological navigation statistics.\n\n"
            "Reads an SQLite database produced by the topological navigation "
            "server and provides commands for inspecting stored maps and "
            "traversal statistics."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument("database", help="Path to the SQLite database file.")

    sub = p.add_subparsers(dest="command", metavar="<command>")
    sub.required = True

    # ------------------------------------------------------------------
    # 'map' command
    # ------------------------------------------------------------------
    map_p = sub.add_parser(
        "map",
        help="Operations on stored topological maps.",
        description="Manage topological maps stored in the stats database.",
    )
    map_sub = map_p.add_subparsers(dest="map_subcommand", metavar="<sub-command>")
    map_sub.required = True

    # map list
    map_sub.add_parser(
        "list",
        help="List all stored maps as a Markdown table.",
    )

    # map show
    show_p = map_sub.add_parser(
        "show",
        help="Show metadata for a specific map.",
    )
    show_p.add_argument(
        "identifier",
        help="Map name or map hash.",
    )

    # map export
    export_p = map_sub.add_parser(
        "export",
        help="Export a stored map as YAML to stdout.",
    )
    export_p.add_argument(
        "identifier",
        help="Map name or map hash.",
    )

    # map import
    import_p = map_sub.add_parser(
        "import",
        help="Import a map YAML file into the database.",
    )
    import_p.add_argument(
        "yaml_file",
        help="Path to the YAML map file.",
    )

    # map rm
    rm_p = map_sub.add_parser(
        "rm",
        help="Delete a stored map from the database.",
    )
    rm_p.add_argument(
        "identifier",
        help="Map name or map hash.",
    )

    # map stats
    stats_p = map_sub.add_parser(
        "stats",
        help=(
            "Calculate road-network statistics for a stored map "
            "(nodes, edges, total length, bounding-box area)."
        ),
    )
    stats_p.add_argument(
        "identifier",
        help="Map name or map hash.",
    )

    # ------------------------------------------------------------------
    # 'traversals' command
    # ------------------------------------------------------------------
    trav_p = sub.add_parser(
        "traversals",
        help="Operations on recorded traversal statistics.",
        description=(
            "Analyse traversal statistics stored in the database.\n\n"
            "The optional --filter flag accepts any SQL WHERE expression "
            "applied to the traversals table.  Filter examples:\n"
            "  --filter \"start_time > '2024-01-01'\"\n"
            "  --filter \"status = 'success'\"\n"
            "  --filter \"start_time BETWEEN '2024-01-01' AND '2024-06-01'\"\n"
            "  --filter \"is_segment = 1\""
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    trav_sub = trav_p.add_subparsers(
        dest="trav_subcommand", metavar="<sub-command>",
    )
    trav_sub.required = True

    _filter_kwargs = dict(
        metavar="SQL_EXPR",
        default="",
        help=(
            "Optional SQL WHERE expression to restrict traversal records. "
            "Example: \"start_time > '2024-01-01' AND status = 'success'\""
        ),
    )

    # traversals summary
    summary_p = trav_sub.add_parser(
        "summary",
        help="Overview of traversal counts per map.",
    )
    summary_p.add_argument("--filter", **_filter_kwargs)

    # traversals edge_stats
    es_p = trav_sub.add_parser(
        "edge_stats",
        help="Per-edge statistics for a specific map (YAML output).",
        description=(
            "Calculate detailed statistics for each edge of the specified map.\n\n"
            "Filter examples:\n"
            "  --filter \"status = 'success'\"\n"
            "  --filter \"start_time > '2024-01-01'\"\n"
            "  --filter \"is_segment = 0\""
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    es_p.add_argument("map_id", help="Map name or map hash.")
    es_p.add_argument("--filter", **_filter_kwargs)

    # traversals map_stats
    ms_p = trav_sub.add_parser(
        "map_stats",
        help="Whole-map traversal statistics (YAML output).",
        description=(
            "Calculate statistics across all edges of the specified map.\n\n"
            "Filter examples:\n"
            "  --filter \"start_time BETWEEN '2024-01-01' AND '2024-06-01'\"\n"
            "  --filter \"status != 'aborted'\"\n\n"
            "Top-N options (omit to skip that section):\n"
            "  --topn_failures=5  Top 5 edges by failure count\n"
            "  --topn_success=5   Top 5 edges by success count\n"
            "  --topn_aborted=5   Top 5 edges by aborted count"
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ms_p.add_argument("map_id", help="Map name or map hash.")
    ms_p.add_argument("--filter", **_filter_kwargs)
    ms_p.add_argument(
        "--topn_failures",
        type=int,
        default=None,
        metavar="N",
        help="Report top N edges with the highest failure count.",
    )
    ms_p.add_argument(
        "--topn_success",
        type=int,
        default=None,
        metavar="N",
        help="Report top N edges with the highest success count.",
    )
    ms_p.add_argument(
        "--topn_aborted",
        type=int,
        default=None,
        metavar="N",
        help="Report top N edges with the highest aborted count.",
    )

    return p


# ---------------------------------------------------------------------------
# Dispatch
# ---------------------------------------------------------------------------

_MAP_DISPATCH = {
    "list": cmd_map_list,
    "show": cmd_map_show,
    "export": cmd_map_export,
    "import": cmd_map_import,
    "rm": cmd_map_rm,
    "stats": cmd_map_stats,
}

_TRAV_DISPATCH = {
    "summary": cmd_traversals_summary,
    "edge_stats": cmd_traversals_edge_stats,
    "map_stats": cmd_traversals_map_stats,
}


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
        if args.command == "map":
            handler = _MAP_DISPATCH.get(args.map_subcommand)
            if handler is None:
                parser.print_help()
                sys.exit(1)
            rc = handler(db, args)

        elif args.command == "traversals":
            handler = _TRAV_DISPATCH.get(args.trav_subcommand)
            if handler is None:
                parser.print_help()
                sys.exit(1)
            rc = handler(db, args)

        else:
            parser.print_help()
            rc = 1

    except Exception as exc:
        print("ERROR: %s" % exc, file=sys.stderr)
        rc = 1
    finally:
        db.close()

    sys.exit(rc)


if __name__ == "__main__":
    main()
