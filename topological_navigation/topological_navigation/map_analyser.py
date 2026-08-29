#!/usr/bin/env python3
"""Standalone topological map analyser.

Analyses a ``.tmap2.yaml`` topological map file and reports:

* schema compliance (delegated to :mod:`topological_navigation.validate_map`)
* orphaned nodes (nodes with no incoming edges, i.e. unreachable)
* disconnected sub-maps (weakly connected components)
* map statistics (node/edge counts, total/average edge length, action
  type breakdown, bidirectional edge count, etc.)
* overlapping influence zones (node polygons that overlap, or a node
  whose position falls inside another node's polygon)

It can also render an SVG representation of the whole map, with
bidirectional edges drawn without arrow heads, directional edges drawn
with an arrow head indicating direction, and edges colour-coded by
their action name.

This tool has no ROS 2 runtime dependency: it only needs ``pyyaml``,
``jsonschema`` and ``networkx`` and can therefore be used in CI (e.g. a
GitHub Actions workflow) without a ROS 2 environment.

Usage::

    python3 map_analyser.py analyse map.tmap2.yaml [--svg out.svg]
    python3 map_analyser.py check map.tmap2.yaml
    python3 map_analyser.py svg map.tmap2.yaml -o out.svg

Exit codes for the ``check`` command:
    0 - Map is valid
    1 - Map is invalid
    2 - File not found or other error
"""

import argparse
import itertools
import math
import os
import sys
from collections import Counter
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Set, Tuple

try:
    import networkx as nx
except ImportError:
    print("Error: networkx is required. Install with: pip install networkx")
    sys.exit(2)

from topological_navigation.networkx_utils import build_graph_from_tmap
from topological_navigation.tmap_utils import load_tmap2_file
from topological_navigation.validate_map import validate_map


Point = Tuple[float, float]

# Deterministic, high-contrast colour palette used to colour-code edges by
# their action name.  Falls back to a hash-based colour for unknown actions
# so that any number of custom action names still gets a stable colour.
_ACTION_COLOURS = {
    "navigate_to_pose": "#1f77b4",
    "row_traversal": "#2ca02c",
    "goal_align": "#ff7f0e",
    "dock": "#9467bd",
    "undock": "#8c564b",
}
_FALLBACK_PALETTE = [
    "#e377c2", "#7f7f7f", "#bcbd22", "#17becf", "#d62728",
]


def _colour_for_action(action: str) -> str:
    """Return a stable hex colour for an action name."""
    if action in _ACTION_COLOURS:
        return _ACTION_COLOURS[action]
    index = sum(ord(c) for c in action) % len(_FALLBACK_PALETTE)
    return _FALLBACK_PALETTE[index]


# =====================================================================
# Geometry helpers
# =====================================================================

def get_node_polygon(graph: "nx.DiGraph", node_name: str) -> List[Point]:
    """Return the absolute (x, y) polygon vertices of a node's influence zone.

    Influence zone vertices are stored relative to the node position (see
    :func:`topological_navigation.networkx_utils.build_graph_from_tmap`), so
    they are translated by the node's (x, y) position here. Returns an
    empty list if the node has no ``verts`` defined.
    """
    attrs = graph.nodes[node_name]
    x, y = attrs.get("x", 0.0), attrs.get("y", 0.0)
    verts = attrs.get("verts", []) or []
    return [(x + v["x"], y + v["y"]) for v in verts]


def _ccw(a: Point, b: Point, c: Point) -> bool:
    return (c[1] - a[1]) * (b[0] - a[0]) > (c[0] - a[0]) * (b[1] - a[1])


def _segments_intersect(a1: Point, a2: Point, b1: Point, b2: Point) -> bool:
    """Return True if segment a1-a2 properly crosses segment b1-b2."""
    return (
        _ccw(a1, b1, b2) != _ccw(a2, b1, b2)
        and _ccw(a1, a2, b1) != _ccw(a1, a2, b2)
    )


def point_in_polygon(point: Point, polygon: List[Point]) -> bool:
    """Ray-casting point-in-polygon test for a generic simple polygon."""
    if len(polygon) < 3:
        return False

    x, y = point
    inside = False
    n = len(polygon)
    p1x, p1y = polygon[0]
    for i in range(n + 1):
        p2x, p2y = polygon[i % n]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xints = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xints:
                        inside = not inside
        p1x, p1y = p2x, p2y
    return inside


def polygons_overlap(poly_a: List[Point], poly_b: List[Point]) -> bool:
    """Return True if two (possibly non-convex) simple polygons overlap.

    Detects both edge-crossing overlaps and full containment (one polygon
    entirely inside the other, which produces no edge crossings).
    """
    if len(poly_a) < 3 or len(poly_b) < 3:
        return False

    for i in range(len(poly_a)):
        a1, a2 = poly_a[i], poly_a[(i + 1) % len(poly_a)]
        for j in range(len(poly_b)):
            b1, b2 = poly_b[j], poly_b[(j + 1) % len(poly_b)]
            if _segments_intersect(a1, a2, b1, b2):
                return True

    if point_in_polygon(poly_a[0], poly_b) or point_in_polygon(poly_b[0], poly_a):
        return True

    return False


def find_overlapping_influence_zones(graph: "nx.DiGraph") -> List[Dict[str, Any]]:
    """Find pairs of nodes whose influence zone polygons overlap.

    Also flags cases where a node's *position* falls inside another
    node's influence zone polygon, even if the polygons themselves do
    not otherwise overlap (e.g. missing/degenerate polygon on the first
    node).

    Returns:
        List of dicts: ``{'node_a': str, 'node_b': str, 'reason': str}``.
    """
    overlaps: List[Dict[str, Any]] = []
    node_names = list(graph.nodes())
    polygons = {n: get_node_polygon(graph, n) for n in node_names}

    for node_a, node_b in itertools.combinations(node_names, 2):
        poly_a, poly_b = polygons[node_a], polygons[node_b]
        reasons = []

        if poly_a and poly_b and polygons_overlap(poly_a, poly_b):
            reasons.append("influence zones overlap")
        else:
            pos_a = (graph.nodes[node_a]["x"], graph.nodes[node_a]["y"])
            pos_b = (graph.nodes[node_b]["x"], graph.nodes[node_b]["y"])
            if poly_b and point_in_polygon(pos_a, poly_b):
                reasons.append(f"'{node_a}' is inside '{node_b}' influence zone")
            if poly_a and point_in_polygon(pos_b, poly_a):
                reasons.append(f"'{node_b}' is inside '{node_a}' influence zone")

        if reasons:
            overlaps.append({
                "node_a": node_a,
                "node_b": node_b,
                "reason": "; ".join(reasons),
            })

    return overlaps


# =====================================================================
# Graph analysis
# =====================================================================

def find_orphaned_nodes(graph: "nx.DiGraph") -> List[str]:
    """Return nodes with no incoming edges (unreachable from any other node)."""
    return sorted(n for n in graph.nodes() if graph.in_degree(n) == 0)


def find_disconnected_components(graph: "nx.DiGraph") -> List[Set[str]]:
    """Return the weakly connected components of the graph.

    A single-element list means the map is fully connected. More than
    one component indicates disconnected sub-maps.
    """
    return [set(c) for c in nx.weakly_connected_components(graph)]


def is_bidirectional_edge(graph: "nx.DiGraph", u: str, v: str) -> bool:
    """Return True if edges u->v and v->u both exist."""
    return graph.has_edge(u, v) and graph.has_edge(v, u)


def _edge_length(graph: "nx.DiGraph", u: str, v: str) -> float:
    ux, uy = graph.nodes[u]["x"], graph.nodes[u]["y"]
    vx, vy = graph.nodes[v]["x"], graph.nodes[v]["y"]
    return math.hypot(vx - ux, vy - uy)


def compute_statistics(graph: "nx.DiGraph") -> Dict[str, Any]:
    """Gather summary statistics for the map graph."""
    num_nodes = graph.number_of_nodes()
    num_edges = graph.number_of_edges()

    lengths = [_edge_length(graph, u, v) for u, v in graph.edges()]
    total_length = sum(lengths)

    action_counts: Counter = Counter()
    for _, _, data in graph.edges(data=True):
        action_counts[data.get("action", "unknown")] += 1

    seen = set()
    bidirectional_count = 0
    for u, v in graph.edges():
        if (v, u) in seen:
            continue
        if is_bidirectional_edge(graph, u, v):
            bidirectional_count += 1
        seen.add((u, v))

    components = find_disconnected_components(graph)

    return {
        "num_nodes": num_nodes,
        "num_edges": num_edges,
        "total_edge_length": total_length,
        "average_edge_length": (total_length / num_edges) if num_edges else 0.0,
        "action_counts": dict(action_counts),
        "bidirectional_edge_count": bidirectional_count,
        "unidirectional_edge_count": num_edges - 2 * bidirectional_count,
        "num_connected_components": len(components),
        "orphaned_node_count": len(find_orphaned_nodes(graph)),
    }


# =====================================================================
# SVG generation
# =====================================================================

def generate_svg(
    graph: "nx.DiGraph",
    output_path: str,
    title: Optional[str] = None,
    width: int = 1200,
    height: int = 900,
    margin: float = 40.0,
) -> str:
    """Render the topological map graph as an SVG file.

    Bidirectional edges (edge exists in both directions) are drawn as
    plain lines with no arrow head. Directional edges are drawn with an
    arrow head indicating the direction of travel. Edges are
    colour-coded by their ``action`` name.

    Args:
        graph: Graph built by :func:`build_graph_from_tmap`.
        output_path: Path to write the ``.svg`` file to.
        title: Optional title rendered at the top of the image.
        width, height: Output image size in pixels.
        margin: Margin in pixels around the map content.

    Returns:
        The SVG document as a string (also written to ``output_path``).
    """
    node_names = list(graph.nodes())
    if not node_names:
        raise ValueError("Cannot render SVG for an empty map")

    xs = [graph.nodes[n]["x"] for n in node_names]
    ys = [graph.nodes[n]["y"] for n in node_names]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)

    span_x = max(max_x - min_x, 1e-6)
    span_y = max(max_y - min_y, 1e-6)
    drawable_w = width - 2 * margin
    drawable_h = height - 2 * margin
    scale = min(drawable_w / span_x, drawable_h / span_y)

    def to_px(point: Point) -> Point:
        px = margin + (point[0] - min_x) * scale
        # SVG y grows downward; flip so the map is drawn "north up".
        py = margin + (max_y - point[1]) * scale
        return px, py

    svg_parts: List[str] = []
    svg_parts.append(
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" '
        f'height="{height}" viewBox="0 0 {width} {height}">'
    )
    svg_parts.append('<rect width="100%" height="100%" fill="white"/>')

    if title:
        svg_parts.append(
            f'<text x="{width / 2:.1f}" y="20" text-anchor="middle" '
            f'font-size="16" font-family="sans-serif" font-weight="bold">'
            f'{title}</text>'
        )

    # Arrow-head markers, one per colour used by a directional edge.
    used_colours = sorted({
        _colour_for_action(data.get("action", "unknown"))
        for u, v, data in graph.edges(data=True)
        if not is_bidirectional_edge(graph, u, v)
    })
    svg_parts.append('<defs>')
    for colour in used_colours:
        marker_id = f"arrow-{colour.lstrip('#')}"
        svg_parts.append(
            f'<marker id="{marker_id}" viewBox="0 0 10 10" refX="9" refY="5" '
            f'markerWidth="6" markerHeight="6" orient="auto-start-reverse">'
            f'<path d="M 0 0 L 10 5 L 0 10 z" fill="{colour}"/>'
            f'</marker>'
        )
    svg_parts.append('</defs>')

    # --- edges -----------------------------------------------------------
    drawn: Set[Tuple[str, str]] = set()
    for u, v, data in graph.edges(data=True):
        if (v, u) in drawn:
            continue
        drawn.add((u, v))

        action = data.get("action", "unknown")
        colour = _colour_for_action(action)
        bidirectional = is_bidirectional_edge(graph, u, v)

        x1, y1 = to_px((graph.nodes[u]["x"], graph.nodes[u]["y"]))
        x2, y2 = to_px((graph.nodes[v]["x"], graph.nodes[v]["y"]))

        marker_attr = ""
        if not bidirectional:
            marker_id = f"arrow-{colour.lstrip('#')}"
            marker_attr = f' marker-end="url(#{marker_id})"'

        svg_parts.append(
            f'<line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" '
            f'stroke="{colour}" stroke-width="2"{marker_attr}>'
            f'<title>{data.get("edge_id", "")} ({action})</title>'
            f'</line>'
        )

    # --- nodes -------------------------------------------------------------
    for node_name in node_names:
        px, py = to_px((graph.nodes[node_name]["x"], graph.nodes[node_name]["y"]))
        svg_parts.append(
            f'<circle cx="{px:.2f}" cy="{py:.2f}" r="6" fill="steelblue" '
            f'stroke="black" stroke-width="1"><title>{node_name}</title></circle>'
        )
        svg_parts.append(
            f'<text x="{px + 8:.2f}" y="{py - 8:.2f}" font-size="10" '
            f'font-family="sans-serif">{node_name}</text>'
        )

    # --- legend --------------------------------------------------------
    actions = sorted({data.get("action", "unknown") for _, _, data in graph.edges(data=True)})
    legend_y = height - 20 * len(actions) - 10
    for i, action in enumerate(actions):
        colour = _colour_for_action(action)
        ly = legend_y + i * 20
        svg_parts.append(
            f'<line x1="{width - 180}" y1="{ly}" x2="{width - 150}" y2="{ly}" '
            f'stroke="{colour}" stroke-width="3"/>'
        )
        svg_parts.append(
            f'<text x="{width - 145}" y="{ly + 4:.1f}" font-size="11" '
            f'font-family="sans-serif">{action}</text>'
        )

    svg_parts.append('</svg>')
    svg_doc = "\n".join(svg_parts)

    with open(output_path, "w", encoding="utf-8") as fh:
        fh.write(svg_doc)

    return svg_doc


# =====================================================================
# Top-level analysis
# =====================================================================

@dataclass
class AnalysisResult:
    """Aggregated results of analysing a topological map."""

    map_file: str
    schema_valid: bool
    schema_message: str
    orphaned_nodes: List[str] = field(default_factory=list)
    disconnected_components: List[Set[str]] = field(default_factory=list)
    statistics: Dict[str, Any] = field(default_factory=dict)
    overlaps: List[Dict[str, Any]] = field(default_factory=list)
    svg_path: Optional[str] = None

    @property
    def is_valid(self) -> bool:
        """A map is valid if it is schema-compliant, has no orphaned nodes,
        and has no overlapping influence zones. Disconnected sub-maps are
        acceptable and only reported as a warning."""
        return (
            self.schema_valid
            and not self.orphaned_nodes
            and not self.overlaps
        )

    def format_report(self) -> str:
        """Return a human-readable, multi-section analysis report."""
        lines = [f"Map analysis report: {self.map_file}", "=" * 60]

        lines.append("")
        lines.append("[Schema validation]")
        lines.append(("  PASS" if self.schema_valid else "  FAIL") + f": {self.schema_message}")

        lines.append("")
        lines.append("[Orphaned nodes]")
        if self.orphaned_nodes:
            lines.append(f"  FAIL: {len(self.orphaned_nodes)} orphaned node(s) found:")
            for n in self.orphaned_nodes:
                lines.append(f"    - {n}")
        else:
            lines.append("  PASS: No orphaned nodes found")

        lines.append("")
        lines.append("[Disconnected sub-maps]")
        if len(self.disconnected_components) > 1:
            lines.append(
                f"  WARNING: {len(self.disconnected_components)} disconnected "
                "sub-map(s) found:"
            )
            for i, comp in enumerate(self.disconnected_components, start=1):
                lines.append(f"    - sub-map {i}: {sorted(comp)}")
        else:
            lines.append("  PASS: Map is fully connected")

        lines.append("")
        lines.append("[Overlapping influence zones]")
        if self.overlaps:
            lines.append(f"  FAIL: {len(self.overlaps)} overlap(s) found:")
            for o in self.overlaps:
                lines.append(f"    - {o['node_a']} <-> {o['node_b']}: {o['reason']}")
        else:
            lines.append("  PASS: No overlapping influence zones found")

        lines.append("")
        lines.append("[Statistics]")
        stats = self.statistics
        lines.append(f"  Nodes: {stats.get('num_nodes', 0)}")
        lines.append(f"  Edges: {stats.get('num_edges', 0)}")
        lines.append(f"  Total edge length: {stats.get('total_edge_length', 0.0):.2f} m")
        lines.append(f"  Average edge length: {stats.get('average_edge_length', 0.0):.2f} m")
        lines.append(f"  Bidirectional edges: {stats.get('bidirectional_edge_count', 0)}")
        lines.append(f"  Unidirectional edges: {stats.get('unidirectional_edge_count', 0)}")
        lines.append("  Actions:")
        for action, count in sorted(stats.get("action_counts", {}).items()):
            lines.append(f"    - {action}: {count}")

        if self.svg_path:
            lines.append("")
            lines.append(f"[SVG] Generated at {self.svg_path}")

        return "\n".join(lines)


def analyse_map(
    map_file: str,
    schema_file: Optional[str] = None,
    svg_path: Optional[str] = None,
) -> AnalysisResult:
    """Run the full map analysis pipeline and return an :class:`AnalysisResult`."""
    is_valid, message = validate_map(map_file, schema_file)

    tmap_data = load_tmap2_file(map_file)
    graph = build_graph_from_tmap(tmap_data)

    if graph is None:
        return AnalysisResult(
            map_file=map_file,
            schema_valid=is_valid,
            schema_message=message,
            orphaned_nodes=[],
            disconnected_components=[],
            statistics={},
            overlaps=[],
        )

    result = AnalysisResult(
        map_file=map_file,
        schema_valid=is_valid,
        schema_message=message,
        orphaned_nodes=find_orphaned_nodes(graph),
        disconnected_components=find_disconnected_components(graph),
        statistics=compute_statistics(graph),
        overlaps=find_overlapping_influence_zones(graph),
    )

    if svg_path:
        generate_svg(graph, svg_path, title=os.path.basename(map_file))
        result.svg_path = svg_path

    return result


# =====================================================================
# CLI
# =====================================================================

def _add_common_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("map_file", help="Path to the topological map YAML file")
    parser.add_argument("--schema", "-s", help="Path to the schema YAML file (optional)")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Analyse topological map (.tmap2.yaml) files.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s analyse my_map.tmap2.yaml
  %(prog)s analyse my_map.tmap2.yaml --svg my_map.svg
  %(prog)s check my_map.tmap2.yaml
  %(prog)s svg my_map.tmap2.yaml -o my_map.svg
        """,
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    analyse_parser = subparsers.add_parser(
        "analyse", aliases=["analyze"], help="Run full analysis and print a report"
    )
    _add_common_args(analyse_parser)
    analyse_parser.add_argument("--svg", help="Also generate an SVG rendering of the map")

    check_parser = subparsers.add_parser(
        "check",
        help="Check map validity; exit code reflects the result (for CI use)",
    )
    _add_common_args(check_parser)
    check_parser.add_argument("--svg", help="Also generate an SVG rendering of the map")

    svg_parser = subparsers.add_parser("svg", help="Generate an SVG rendering of the map")
    _add_common_args(svg_parser)
    svg_parser.add_argument("--output", "-o", required=True, help="Output SVG file path")

    return parser


def main(argv: Optional[List[str]] = None) -> None:
    parser = build_arg_parser()
    args = parser.parse_args(argv)

    if not os.path.isfile(args.map_file):
        print(f"Error: Map file not found: {args.map_file}")
        sys.exit(2)

    svg_path = getattr(args, "svg", None) or getattr(args, "output", None)

    try:
        result = analyse_map(args.map_file, args.schema, svg_path=svg_path)
    except Exception as exc:  # noqa: BLE001 - report any load/parsing error to the user
        print(f"Error analysing map: {exc}")
        sys.exit(2)

    if args.command in ("analyse", "analyze", "check"):
        print(result.format_report())

    if args.command == "check":
        if result.is_valid:
            print("\n✓ Map is VALID")
            sys.exit(0)
        else:
            print("\n✗ Map is INVALID")
            sys.exit(1)

    if args.command == "svg":
        print(f"SVG written to {result.svg_path}")


if __name__ == "__main__":
    main()
