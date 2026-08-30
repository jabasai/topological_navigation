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

Finally, it can produce a smaller, semantically identical copy of a map
by collapsing repeated node/edge sub-structures (e.g. shared
``properties``, ``verts`` or ``orientation`` dicts) into named YAML
anchors, and/or by dropping nodes/edges unreachable from a given node.

This tool has no ROS 2 runtime dependency: it only needs ``pyyaml``,
``jsonschema`` and ``networkx`` (plus ``numpy`` and ``scipy`` via
:mod:`topological_navigation.networkx_utils`) and can therefore be used in CI (e.g. a
GitHub Actions workflow) without a ROS 2 environment.

Usage::

    python3 map_analyser.py analyse map.tmap2.yaml [--svg out.svg]
    python3 map_analyser.py check map.tmap2.yaml
    python3 map_analyser.py svg map.tmap2.yaml -o out.svg
    python3 map_analyser.py minify map.tmap2.yaml [--anchors] [--strip-unreachable NODE]

Each check can be individually turned off or have its severity changed
between "warning" (printed, exit code unaffected) and "error" (printed,
exit code 1) via ``--<check>={false,warning,error}`` switches, e.g.
``--sub-map-separation=error`` or ``--influence-zone-overlap=false``.
Defaults: schema/orphaned-node = error, sub-map-separation/
influence-zone-overlap = warning.

Exit codes for the ``check`` command:
    0 - Map is valid
    1 - Map is invalid
    2 - File not found or other error
"""

import argparse
import itertools
import logging
import math
import os
import re
import sys
from collections import Counter, defaultdict
from copy import deepcopy
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Set, Tuple
from xml.sax.saxutils import escape as _xml_escape

try:
    import networkx as nx
except ImportError:
    print("Error: networkx is required. Install with: pip install networkx")
    sys.exit(2)

try:
    import yaml
except ImportError:
    print("Error: PyYAML is required. Install with: pip install pyyaml")
    sys.exit(2)

from topological_navigation.networkx_utils import build_graph_from_tmap
from topological_navigation.tmap_utils import CustomSafeLoader, NoAliasDumper, load_tmap2_file
from topological_navigation.validate_map import validate_map

_LOGGER = logging.getLogger(__name__)


Point = Tuple[float, float]

# Identifiers for the individual "check" tests, and the severity each one
# is treated with by default when computing overall map validity:
#   "error"   - a failure makes the map INVALID (non-zero exit code)
#   "warning" - a failure is printed but does not affect validity/exit code
#   None      - the check is disabled entirely (not run, not printed)
DEFAULT_CHECK_SEVERITY: Dict[str, Optional[str]] = {
    "schema": "error",
    "orphaned-node": "error",
    "sub-map-separation": "warning",
    "influence-zone-overlap": "warning",
}


def _parse_severity(value: str) -> Optional[str]:
    """Parse a ``--<check>=<value>`` CLI value into a check severity."""
    normalised = str(value).strip().lower()
    if normalised in ("false", "off", "disable", "disabled", "none"):
        return None
    if normalised in ("warning", "warn"):
        return "warning"
    if normalised in ("error", "true", "on"):
        return "error"
    raise argparse.ArgumentTypeError(
        f"invalid severity {value!r} (expected one of: false, warning, error)"
    )

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
    action = str(action) if action is not None else "unknown"
    if action in _ACTION_COLOURS:
        return _ACTION_COLOURS[action]
    index = sum(ord(c) for c in action) % len(_FALLBACK_PALETTE)
    return _FALLBACK_PALETTE[index]


def _svg_text(value: Any) -> str:
    """Escape *value* for safe inclusion as SVG/XML element text content.

    Any user-controlled string (node names, action names, edge ids, map
    titles, ...) must be escaped before being embedded in generated SVG
    markup, otherwise characters such as ``&``, ``<`` or ``>`` would
    produce invalid/unparseable XML.
    """
    return _xml_escape("" if value is None else str(value))


def _finite_or(value: Any, default: float) -> float:
    """Coerce *value* to a finite float, falling back to *default* otherwise."""
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    if math.isnan(number) or math.isinf(number):
        return default
    return number


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

    # Coerce every node coordinate to a finite float. Missing/NaN/Inf
    # coordinates would otherwise propagate into the generated markup as
    # literal "nan"/"inf" attribute values, which is not valid SVG.
    xs = [_finite_or(graph.nodes[n].get("x"), 0.0) for n in node_names]
    ys = [_finite_or(graph.nodes[n].get("y"), 0.0) for n in node_names]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)

    # Guard against degenerate output sizes (e.g. width/height smaller than
    # 2x margin) which would otherwise make the map area negative and
    # produce a non-finite or negative scale factor.
    width = max(int(_finite_or(width, 1200)), 1)
    height = max(int(_finite_or(height, 900)), 1)
    margin = max(_finite_or(margin, 40.0), 0.0)
    margin = min(margin, (width - 1) / 2.0, (height - 1) / 2.0)
    margin = max(margin, 0.0)

    span_x = max(max_x - min_x, 1e-6)
    span_y = max(max_y - min_y, 1e-6)
    drawable_w = width - 2 * margin
    drawable_h = height - 2 * margin
    scale = min(drawable_w / span_x, drawable_h / span_y)

    # Sanitized (x, y) position per node, reused for every draw call below
    # so that a single NaN/Inf coordinate is normalised once rather than
    # risking re-reading the raw (possibly non-finite) graph attribute.
    positions: Dict[str, Point] = dict(zip(node_names, zip(xs, ys)))

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
            f'{_svg_text(title)}</text>'
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

        x1, y1 = to_px(positions[u])
        x2, y2 = to_px(positions[v])

        marker_attr = ""
        if not bidirectional:
            marker_id = f"arrow-{colour.lstrip('#')}"
            marker_attr = f' marker-end="url(#{marker_id})"'

        svg_parts.append(
            f'<line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" '
            f'stroke="{colour}" stroke-width="2"{marker_attr}>'
            f'<title>{_svg_text(data.get("edge_id", ""))} ({_svg_text(action)})</title>'
            f'</line>'
        )

    # --- nodes -------------------------------------------------------------
    for node_name in node_names:
        px, py = to_px(positions[node_name])
        svg_parts.append(
            f'<circle cx="{px:.2f}" cy="{py:.2f}" r="6" fill="steelblue" '
            f'stroke="black" stroke-width="1"><title>{_svg_text(node_name)}</title></circle>'
        )
        svg_parts.append(
            f'<text x="{px + 8:.2f}" y="{py - 8:.2f}" font-size="10" '
            f'font-family="sans-serif">{_svg_text(node_name)}</text>'
        )

    # --- legend --------------------------------------------------------
    actions = sorted({
        str(data.get("action", "unknown") or "unknown")
        for _, _, data in graph.edges(data=True)
    })
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
            f'font-family="sans-serif">{_svg_text(action)}</text>'
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
    check_severities: Dict[str, Optional[str]] = field(
        default_factory=lambda: dict(DEFAULT_CHECK_SEVERITY)
    )

    def _status_label(self, check_id: str, failed: bool) -> str:
        """Return PASS/WARNING/FAIL/SKIPPED for *check_id* given its severity."""
        severity = self.check_severities.get(check_id, DEFAULT_CHECK_SEVERITY.get(check_id))
        if severity is None:
            return "SKIPPED"
        if not failed:
            return "PASS"
        return "FAIL" if severity == "error" else "WARNING"

    @property
    def is_valid(self) -> bool:
        """A map is valid unless a check whose severity is "error" failed.

        Checks whose severity is "warning" are reported but do not affect
        validity; checks whose severity is ``None`` are disabled entirely.
        """
        failures = {
            "schema": not self.schema_valid,
            "orphaned-node": bool(self.orphaned_nodes),
            "sub-map-separation": len(self.disconnected_components) > 1,
            "influence-zone-overlap": bool(self.overlaps),
        }
        return not any(
            failed and self.check_severities.get(check_id) == "error"
            for check_id, failed in failures.items()
        )

    def format_report(self) -> str:
        """Return a human-readable, multi-section analysis report."""
        lines = [f"Map analysis report: {self.map_file}", "=" * 60]

        lines.append("")
        lines.append("[Schema validation]")
        label = self._status_label("schema", not self.schema_valid)
        if label == "SKIPPED":
            lines.append("  SKIPPED: check disabled")
        else:
            lines.append(f"  {label}: {self.schema_message}")

        lines.append("")
        lines.append("[Orphaned nodes]")
        label = self._status_label("orphaned-node", bool(self.orphaned_nodes))
        if label == "SKIPPED":
            lines.append("  SKIPPED: check disabled")
        elif self.orphaned_nodes:
            lines.append(f"  {label}: {len(self.orphaned_nodes)} orphaned node(s) found:")
            for n in self.orphaned_nodes:
                lines.append(f"    - {n}")
        else:
            lines.append(f"  {label}: No orphaned nodes found")

        lines.append("")
        lines.append("[Disconnected sub-maps]")
        submaps_failed = len(self.disconnected_components) > 1
        label = self._status_label("sub-map-separation", submaps_failed)
        if label == "SKIPPED":
            lines.append("  SKIPPED: check disabled")
        elif submaps_failed:
            lines.append(
                f"  {label}: {len(self.disconnected_components)} disconnected "
                "sub-map(s) found:"
            )
            for i, comp in enumerate(self.disconnected_components, start=1):
                lines.append(f"    - sub-map {i}: {sorted(comp)}")
        else:
            lines.append(f"  {label}: Map is fully connected")

        lines.append("")
        lines.append("[Overlapping influence zones]")
        label = self._status_label("influence-zone-overlap", bool(self.overlaps))
        if label == "SKIPPED":
            lines.append("  SKIPPED: check disabled")
        elif self.overlaps:
            lines.append(f"  {label}: {len(self.overlaps)} overlap(s) found:")
            for o in self.overlaps:
                lines.append(f"    - {o['node_a']} <-> {o['node_b']}: {o['reason']}")
        else:
            lines.append(f"  {label}: No overlapping influence zones found")

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
    check_severities: Optional[Dict[str, Optional[str]]] = None,
) -> AnalysisResult:
    """Run the full map analysis pipeline and return an :class:`AnalysisResult`.

    *check_severities* optionally overrides the default severity (see
    :data:`DEFAULT_CHECK_SEVERITY`) for each check; a severity of ``None``
    disables that check so it is neither run nor reported.
    """
    severities = dict(DEFAULT_CHECK_SEVERITY)
    if check_severities:
        severities.update(check_severities)

    if severities["schema"] is not None:
        is_valid, message = validate_map(map_file, schema_file)
    else:
        is_valid, message = True, "Skipped (check disabled)"

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
            check_severities=severities,
        )

    result = AnalysisResult(
        map_file=map_file,
        schema_valid=is_valid,
        schema_message=message,
        orphaned_nodes=find_orphaned_nodes(graph) if severities["orphaned-node"] is not None else [],
        disconnected_components=(
            find_disconnected_components(graph)
            if severities["sub-map-separation"] is not None
            else []
        ),
        statistics=compute_statistics(graph),
        overlaps=(
            find_overlapping_influence_zones(graph)
            if severities["influence-zone-overlap"] is not None
            else []
        ),
        check_severities=severities,
    )

    if svg_path:
        generate_svg(graph, svg_path, title=os.path.basename(map_file))
        result.svg_path = svg_path

    return result


# =====================================================================
# Minification
# =====================================================================


def _canonical_key(node: Any) -> Any:
    """Hashable, order-independent-for-dicts structural signature of *node*."""
    if isinstance(node, dict):
        return ("dict", tuple(sorted((k, _canonical_key(v)) for k, v in node.items())))
    if isinstance(node, list):
        return ("list", tuple(_canonical_key(v) for v in node))
    return ("scalar", type(node).__name__, node)


def _estimate_size(node: Any) -> int:
    """Cheap, library-independent character-size estimate, used only for thresholding."""
    if isinstance(node, dict):
        return 2 + sum(len(str(k)) + 2 + _estimate_size(v) for k, v in node.items())
    if isinstance(node, list):
        return 2 + sum(_estimate_size(v) + 2 for v in node)
    return len(str(node))


def _collect_signatures(
    node: Any,
    path: List[str],
    counts: Dict[Any, int],
    representative: Dict[Any, Any],
    first_path: Dict[Any, List[str]],
) -> None:
    """Bottom-up walk recording, for every dict/list subtree: its occurrence count,
    a representative instance, and the context path of its first occurrence (the
    latter is used to name anchors sensibly, e.g. "properties" or "verts")."""
    if isinstance(node, dict):
        for k, v in node.items():
            _collect_signatures(v, path + [str(k)], counts, representative, first_path)
        sig = _canonical_key(node)
        counts[sig] += 1
        representative.setdefault(sig, node)
        first_path.setdefault(sig, path)
    elif isinstance(node, list):
        for v in node:
            _collect_signatures(v, path, counts, representative, first_path)
        sig = _canonical_key(node)
        counts[sig] += 1
        representative.setdefault(sig, node)
        first_path.setdefault(sig, path)


def _rebuild_with_anchors(
    node: Any,
    mergeable_sigs: Set[Any],
    representative: Dict[Any, Any],
    cache: Dict[Any, Any],
) -> Any:
    """Rebuild *node*, replacing any mergeable subtree with a single shared object (by
    identity) so the YAML dumper emits it once with an anchor and aliases it elsewhere."""
    if isinstance(node, dict):
        sig = _canonical_key(node)
        if sig in mergeable_sigs:
            if sig not in cache:
                rep = representative[sig]
                cache[sig] = {
                    k: _rebuild_with_anchors(v, mergeable_sigs, representative, cache)
                    for k, v in rep.items()
                }
            return cache[sig]
        return {k: _rebuild_with_anchors(v, mergeable_sigs, representative, cache) for k, v in node.items()}
    if isinstance(node, list):
        sig = _canonical_key(node)
        if sig in mergeable_sigs:
            if sig not in cache:
                rep = representative[sig]
                cache[sig] = [_rebuild_with_anchors(v, mergeable_sigs, representative, cache) for v in rep]
            return cache[sig]
        return [_rebuild_with_anchors(v, mergeable_sigs, representative, cache) for v in node]
    return node


def _kind_for_path(path: List[str]) -> str:
    """Return the most specific context label (last path element) for naming, e.g. 'verts'."""
    return path[-1] if path else "shared"


def _slug(text: Any) -> str:
    """Turn *text* into a short, YAML-key-safe, lowercase identifier fragment."""
    slug = re.sub(r"[^a-zA-Z0-9]+", "_", str(text).strip().lower())
    return slug.strip("_") or "x"


def _fmt_num(value: Any) -> str:
    """Format a number compactly for use inside an anchor name (e.g. 0.3 -> '0_3')."""
    try:
        number = float(value)
    except (TypeError, ValueError):
        return _slug(value)
    if number.is_integer():
        number = int(number)
    return str(number).replace("-", "m").replace(".", "_")


def _is_identity_orientation(value: Dict[str, Any]) -> bool:
    try:
        return (
            abs(float(value.get("w", 0.0)) - 1.0) < 1e-6
            and all(abs(float(value.get(k, 0.0))) < 1e-6 for k in ("x", "y", "z"))
        )
    except (TypeError, ValueError):
        return False


def _suggest_anchor_name(kind: str, value: Any) -> str:
    """Best-effort, content-aware anchor name for a merged subtree; falls back to *kind*."""
    if kind == "orientation" and isinstance(value, dict) and _is_identity_orientation(value):
        return "orientation_identity"
    if kind == "properties" and isinstance(value, dict):
        if "semantics" in value:
            return f"properties_{_slug(value['semantics'])}"
        keys = set(value.keys())
        if keys == {"xy_goal_tolerance", "yaw_goal_tolerance"}:
            return f"properties_tol_{_fmt_num(value['xy_goal_tolerance'])}_{_fmt_num(value['yaw_goal_tolerance'])}"
        if keys == {"max_speed"}:
            return f"properties_speed_{_fmt_num(value['max_speed'])}"
    if kind == "verts" and isinstance(value, list) and value:
        xs = [v.get("x", 0.0) for v in value if isinstance(v, dict)]
        ys = [v.get("y", 0.0) for v in value if isinstance(v, dict)]
        if xs and ys:
            width = round(max(xs) - min(xs), 2)
            height = round(max(ys) - min(ys), 2)
            return f"verts_{_fmt_num(width)}x{_fmt_num(height)}"
    return kind


def _assign_anchor_names(
    ordered_sigs: List[Any],
    representative: Dict[Any, Any],
    first_path: Dict[Any, List[str]],
) -> Dict[Any, str]:
    """Return {signature: anchor_name}, generating readable, collision-free names."""
    names: Dict[Any, str] = {}
    used: Counter = Counter()
    for sig in ordered_sigs:
        kind = _kind_for_path(first_path.get(sig, []))
        base = _suggest_anchor_name(kind, representative[sig])
        used[base] += 1
        names[sig] = base if used[base] == 1 else f"{base}_{used[base]}"
    return names


def _strip_unreachable(
    tmap_data: Dict[str, Any],
    start_node: str,
    logger: Optional[logging.Logger] = None,
) -> Tuple[Dict[str, Any], int, int]:
    """Drop nodes (and their edges) not reachable via a directed path from *start_node*.

    Returns the modified map dict plus the number of removed nodes and edges. The
    result always passes the ``orphaned-node`` and ``sub-map-separation`` checks for
    every node except (possibly) *start_node* itself, since every kept node is by
    construction reachable from - and thus has an incoming edge from - another kept
    node along its path from *start_node*.
    """
    graph = build_graph_from_tmap(tmap_data)
    if graph is None or start_node not in graph:
        raise ValueError(f"Start node '{start_node}' not found in map")

    keep = {start_node} | nx.descendants(graph, start_node)
    original_node_count = graph.number_of_nodes()

    stripped = deepcopy(tmap_data)
    kept_nodes = [n for n in (stripped.get("nodes") or []) if n["node"]["name"] in keep]

    removed_edges = 0
    for n in kept_nodes:
        edges = n["node"].get("edges") or []
        kept_edges = [e for e in edges if e.get("node") in keep]
        removed_edges += len(edges) - len(kept_edges)
        n["node"]["edges"] = kept_edges

    stripped["nodes"] = kept_nodes
    removed_nodes = original_node_count - len(kept_nodes)

    (logger or _LOGGER).info(
        "strip-unreachable from '%s': kept %d/%d node(s); removed %d node(s) and %d edge(s)",
        start_node, len(kept_nodes), original_node_count, removed_nodes, removed_edges,
    )

    return stripped, removed_nodes, removed_edges


def _make_named_anchor_dumper(anchor_names_by_id: Dict[int, str]):
    """Build a ``SafeDumper`` subclass that emits *anchor_names_by_id*'s names instead
    of PyYAML's default auto-generated ``id001``-style anchor names."""

    class _NamedAnchorDumper(yaml.SafeDumper):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self._anchor_node_names: Dict[int, str] = {}

        def represent_data(self, data):
            node = super().represent_data(data)
            name = anchor_names_by_id.get(id(data))
            if name is not None:
                self._anchor_node_names[id(node)] = name
            return node

        def generate_anchor(self, node):
            return self._anchor_node_names.get(id(node)) or super().generate_anchor(node)

    return _NamedAnchorDumper


def _leading_comment_block(text: str) -> str:
    """Return the leading run of comment/blank lines at the top of *text*, or ""."""
    block: List[str] = []
    for line in text.splitlines(keepends=True):
        stripped = line.strip()
        if stripped == "" or stripped.startswith("#"):
            block.append(line)
        else:
            break
    return "".join(block)


def _derive_output_path(map_file: str) -> str:
    """Derive a sibling `<name>.min.<ext>` output path for *map_file*."""
    directory, base = os.path.split(map_file)
    stem, sep, rest = base.partition(".")
    derived = f"{stem}.min{sep}{rest}" if sep else f"{stem}.min"
    return os.path.join(directory, derived) if directory else derived


@dataclass
class MinifyResult:
    """Outcome of running :func:`minify_map`."""

    map_file: str
    output_file: str
    original_size: int
    minified_size: int
    anchors_created: int = 0
    occurrences_collapsed: int = 0
    stripped_nodes: int = 0
    stripped_edges: int = 0
    schema_valid: Optional[bool] = None
    schema_message: str = ""

    @property
    def bytes_saved(self) -> int:
        return self.original_size - self.minified_size

    @property
    def percent_saved(self) -> float:
        return (100.0 * self.bytes_saved / self.original_size) if self.original_size else 0.0

    def format_report(self) -> str:
        lines = [f"Minify report: {self.map_file} -> {self.output_file}", "=" * 60]
        lines.append(f"  Original size:   {self.original_size:,} bytes")
        lines.append(f"  Minified size:   {self.minified_size:,} bytes")
        lines.append(f"  Reduction:       {self.percent_saved:.1f}% ({self.bytes_saved:,} bytes saved)")
        lines.append(
            f"  Anchors created: {self.anchors_created} "
            f"(covering {self.occurrences_collapsed} occurrence(s))"
        )
        if self.stripped_nodes or self.stripped_edges:
            lines.append(
                f"  Stripped unreachable: {self.stripped_nodes} node(s), {self.stripped_edges} edge(s)"
            )
        if self.schema_valid is not None:
            status = "PASS" if self.schema_valid else "FAIL"
            lines.append(f"  Schema check on output: {status}: {self.schema_message}")
        return "\n".join(lines)


def minify_map(
    map_file: str,
    output_file: Optional[str] = None,
    anchors: bool = True,
    strip_comments: bool = False,
    flowstyle: bool = False,
    min_size: int = 100,
    min_occurrences: int = 5,
    strip_unreachable_from: Optional[str] = None,
    schema_file: Optional[str] = None,
    logger: Optional[logging.Logger] = None,
) -> MinifyResult:
    """Write a smaller, semantically-identical copy of *map_file* and return a
    :class:`MinifyResult` describing the outcome.

    Repeated dict/list sub-structures (e.g. shared ``properties``, ``verts`` or
    ``orientation`` blocks) whose serialised size is >= *min_size* and which occur
    >= *min_occurrences* times are collapsed into named YAML anchors gathered under
    a top-level ``anchors`` section, when *anchors* is True. Only the leading
    top-of-file comment block is preserved (per-node/edge inline comments are not
    preserved by the underlying YAML parser); pass ``strip_comments=True`` to drop
    it too. When *strip_unreachable_from* is given, nodes/edges not reachable via a
    directed path from that node are dropped first.
    """
    log = logger or _LOGGER

    with open(map_file, "r", encoding="utf-8") as fh:
        original_text = fh.read()

    tmap_data = load_tmap2_file(map_file)

    stripped_nodes = stripped_edges = 0
    if strip_unreachable_from:
        tmap_data, stripped_nodes, stripped_edges = _strip_unreachable(
            tmap_data, strip_unreachable_from, logger=log
        )

    anchors_section: Dict[str, Any] = {}
    anchor_names_by_id: Dict[int, str] = {}
    occurrences_collapsed = 0

    if anchors:
        counts: Dict[Any, int] = defaultdict(int)
        representative: Dict[Any, Any] = {}
        first_path: Dict[Any, List[str]] = {}
        _collect_signatures(tmap_data, [], counts, representative, first_path)

        candidates = [
            sig for sig, count in counts.items()
            if count >= min_occurrences and _estimate_size(representative[sig]) >= min_size
        ]
        # Smallest/most-nested subtrees first, so a bigger anchor that happens to
        # contain a smaller one references it cleanly rather than the other way round.
        candidates.sort(key=lambda s: _estimate_size(representative[s]))
        mergeable_set = set(candidates)

        anchor_names = _assign_anchor_names(candidates, representative, first_path)

        cache: Dict[Any, Any] = {}
        for sig in candidates:
            anchors_section[anchor_names[sig]] = _rebuild_with_anchors(
                representative[sig], mergeable_set, representative, cache
            )
        occurrences_collapsed = sum(counts[sig] for sig in candidates)

        rebuilt = _rebuild_with_anchors(tmap_data, mergeable_set, representative, cache)
        final_doc = {"anchors": anchors_section, **rebuilt} if anchors_section else rebuilt
        anchor_names_by_id = {id(v): k for k, v in anchors_section.items()}
    else:
        final_doc = deepcopy(tmap_data)

    dumper = _make_named_anchor_dumper(anchor_names_by_id) if anchors_section else NoAliasDumper
    dumped = yaml.dump(
        final_doc,
        Dumper=dumper,
        default_flow_style=flowstyle,
        sort_keys=False,
        allow_unicode=True,
        width=100000 if flowstyle else 120,
    )

    # Round-trip sanity check: minified YAML must parse back to the same data
    # (aliases transparently expand back out), modulo the extra "anchors" section.
    reparsed = yaml.load(dumped, Loader=CustomSafeLoader)
    reparsed_body = {k: v for k, v in reparsed.items() if k != "anchors"}
    if reparsed_body != tmap_data:
        raise RuntimeError("Minification round-trip check failed: output does not match input data")

    leading_comment = "" if strip_comments else _leading_comment_block(original_text)
    output_text = f"{leading_comment}{dumped}"

    out_path = output_file or _derive_output_path(map_file)
    directory = os.path.dirname(os.path.abspath(out_path))
    if directory:
        os.makedirs(directory, exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as fh:
        fh.write(output_text)

    schema_valid, schema_message = validate_map(out_path, schema_file)

    result = MinifyResult(
        map_file=map_file,
        output_file=out_path,
        original_size=len(original_text),
        minified_size=len(output_text),
        anchors_created=len(anchors_section),
        occurrences_collapsed=occurrences_collapsed,
        stripped_nodes=stripped_nodes,
        stripped_edges=stripped_edges,
        schema_valid=schema_valid,
        schema_message=schema_message,
    )
    log.info(
        "Minified %s -> %s: %d -> %d bytes (%.1f%% saved), %d anchor(s) covering %d occurrence(s)",
        map_file, out_path, result.original_size, result.minified_size,
        result.percent_saved, result.anchors_created, result.occurrences_collapsed,
    )
    return result


# =====================================================================
# CLI
# =====================================================================

def _add_common_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("map_file", help="Path to the topological map YAML file")
    parser.add_argument("--schema", "-s", help="Path to the schema YAML file (optional)")


def _add_check_severity_args(parser: argparse.ArgumentParser) -> None:
    """Add per-check ``{false,warning,error}`` severity override switches."""
    parser.add_argument(
        "--schema-check",
        dest="severity_schema",
        type=_parse_severity,
        default=DEFAULT_CHECK_SEVERITY["schema"],
        metavar="{false,warning,error}",
        help="Severity of schema validation failures (default: %(default)s)",
    )
    parser.add_argument(
        "--orphaned-node",
        dest="severity_orphaned_node",
        type=_parse_severity,
        default=DEFAULT_CHECK_SEVERITY["orphaned-node"],
        metavar="{false,warning,error}",
        help="Severity of orphaned nodes (default: %(default)s)",
    )
    parser.add_argument(
        "--sub-map-separation",
        dest="severity_sub_map_separation",
        type=_parse_severity,
        default=DEFAULT_CHECK_SEVERITY["sub-map-separation"],
        metavar="{false,warning,error}",
        help="Severity of disconnected sub-maps (default: %(default)s)",
    )
    parser.add_argument(
        "--influence-zone-overlap",
        dest="severity_influence_zone_overlap",
        type=_parse_severity,
        default=DEFAULT_CHECK_SEVERITY["influence-zone-overlap"],
        metavar="{false,warning,error}",
        help="Severity of overlapping influence zones (default: %(default)s)",
    )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Analyse topological map (.tmap2.yaml) files.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s analyse my_map.tmap2.yaml
  %(prog)s analyse my_map.tmap2.yaml --svg my_map.svg
  %(prog)s check my_map.tmap2.yaml
  %(prog)s check my_map.tmap2.yaml --sub-map-separation=error
  %(prog)s check my_map.tmap2.yaml --influence-zone-overlap=false
  %(prog)s svg my_map.tmap2.yaml -o my_map.svg
  %(prog)s minify my_map.tmap2.yaml
  %(prog)s minify my_map.tmap2.yaml --strip-unreachable Charging --flowstyle
        """,
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    analyse_parser = subparsers.add_parser(
        "analyse", aliases=["analyze"], help="Run full analysis and print a report"
    )
    _add_common_args(analyse_parser)
    _add_check_severity_args(analyse_parser)
    analyse_parser.add_argument("--svg", help="Also generate an SVG rendering of the map")

    check_parser = subparsers.add_parser(
        "check",
        help="Check map validity; exit code reflects the result (for CI use)",
    )
    _add_common_args(check_parser)
    _add_check_severity_args(check_parser)
    check_parser.add_argument("--svg", help="Also generate an SVG rendering of the map")

    svg_parser = subparsers.add_parser("svg", help="Generate an SVG rendering of the map")
    _add_common_args(svg_parser)
    svg_parser.add_argument("--output", "-o", required=True, help="Output SVG file path")

    minify_parser = subparsers.add_parser(
        "minify",
        help="Write a smaller, semantically-identical copy of the map",
    )
    _add_common_args(minify_parser)
    minify_parser.add_argument(
        "--output", "-o", help="Output file path (default: <name>.min.<ext> next to the input)"
    )
    minify_parser.add_argument(
        "--anchors",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Collapse repeated data structures into named YAML anchors (default: enabled)",
    )
    minify_parser.add_argument(
        "--strip-comments",
        action="store_true",
        help="Also drop the file's leading comment block (default: keep it)",
    )
    minify_parser.add_argument(
        "--flowstyle",
        action="store_true",
        help="Emit compact flow-style YAML instead of block style (default: block style)",
    )
    minify_parser.add_argument(
        "--min-size", type=int, default=100,
        help="Minimum serialised size (chars) of a subtree to qualify for anchoring (default: 100)",
    )
    minify_parser.add_argument(
        "--min-occurrences", type=int, default=5,
        help="Minimum number of repeats required for a subtree to qualify for anchoring (default: 5)",
    )
    minify_parser.add_argument(
        "--strip-unreachable",
        metavar="NODE",
        help="Also drop nodes/edges not reachable via a directed path from NODE",
    )

    return parser


def main(argv: Optional[List[str]] = None) -> None:
    logging.basicConfig(level=logging.INFO, format="%(message)s")

    parser = build_arg_parser()
    args = parser.parse_args(argv)

    if not os.path.isfile(args.map_file):
        print(f"Error: Map file not found: {args.map_file}")
        sys.exit(2)

    if args.command == "minify":
        try:
            result = minify_map(
                args.map_file,
                output_file=args.output,
                anchors=args.anchors,
                strip_comments=args.strip_comments,
                flowstyle=args.flowstyle,
                min_size=args.min_size,
                min_occurrences=args.min_occurrences,
                strip_unreachable_from=args.strip_unreachable,
                schema_file=args.schema,
            )
        except Exception as exc:  # noqa: BLE001 - report any load/parsing error to the user
            print(f"Error minifying map: {exc}")
            sys.exit(2)
        print(result.format_report())
        return

    svg_path = getattr(args, "svg", None) or getattr(args, "output", None)

    check_severities = None
    if hasattr(args, "severity_schema"):
        check_severities = {
            "schema": args.severity_schema,
            "orphaned-node": args.severity_orphaned_node,
            "sub-map-separation": args.severity_sub_map_separation,
            "influence-zone-overlap": args.severity_influence_zone_overlap,
        }

    try:
        result = analyse_map(
            args.map_file, args.schema, svg_path=svg_path, check_severities=check_severities
        )
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
