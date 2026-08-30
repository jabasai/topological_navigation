"""Unit tests for the map_analyser module.

Covers geometry helpers (polygon overlap / point-in-polygon), graph
analysis (orphaned nodes, disconnected components, statistics),
SVG generation, and the top-level ``analyse_map`` / CLI ``check``
behaviour.
"""

import math
import os

import networkx as nx
import pytest

import yaml

from topological_navigation.map_analyser import (
    AnalysisResult,
    MinifyResult,
    analyse_map,
    build_arg_parser,
    compute_statistics,
    find_disconnected_components,
    find_orphaned_nodes,
    find_overlapping_influence_zones,
    generate_svg,
    get_node_polygon,
    is_bidirectional_edge,
    main,
    minify_map,
    point_in_polygon,
    polygons_overlap,
)
from topological_navigation.tmap_utils import CustomSafeLoader, load_tmap2_file


FIXTURES = os.path.join(os.path.dirname(__file__), "fixtures")
SIMPLE_MAP = os.path.join(FIXTURES, "simple_map.yaml")
COMPLEX_MAP = os.path.join(FIXTURES, "complex_map.yaml")
MIXED_ACTIONS_MAP = os.path.join(FIXTURES, "mixed_actions_map.yaml")
POLYGON_SHAPES_MAP = os.path.join(FIXTURES, "polygon_shapes_map.yaml")


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

class TestPointInPolygon:
    def test_point_inside_square(self):
        square = [(-1, -1), (1, -1), (1, 1), (-1, 1)]
        assert point_in_polygon((0, 0), square) is True

    def test_point_outside_square(self):
        square = [(-1, -1), (1, -1), (1, 1), (-1, 1)]
        assert point_in_polygon((5, 5), square) is False

    def test_degenerate_polygon_returns_false(self):
        assert point_in_polygon((0, 0), [(0, 0), (1, 1)]) is False


class TestPolygonsOverlap:
    def test_overlapping_squares(self):
        a = [(-1, -1), (1, -1), (1, 1), (-1, 1)]
        b = [(0, 0), (2, 0), (2, 2), (0, 2)]
        assert polygons_overlap(a, b) is True

    def test_disjoint_squares(self):
        a = [(-1, -1), (1, -1), (1, 1), (-1, 1)]
        b = [(10, 10), (12, 10), (12, 12), (10, 12)]
        assert polygons_overlap(a, b) is False

    def test_fully_contained_polygon(self):
        outer = [(-5, -5), (5, -5), (5, 5), (-5, 5)]
        inner = [(-1, -1), (1, -1), (1, 1), (-1, 1)]
        assert polygons_overlap(outer, inner) is True

    def test_empty_polygon_never_overlaps(self):
        square = [(-1, -1), (1, -1), (1, 1), (-1, 1)]
        assert polygons_overlap(square, []) is False


class TestGetNodePolygon:
    def test_translates_verts_by_node_position(self):
        graph = nx.DiGraph()
        graph.add_node(
            "N1", x=10.0, y=5.0,
            verts=[{"x": -1.0, "y": -1.0}, {"x": 1.0, "y": -1.0},
                   {"x": 1.0, "y": 1.0}, {"x": -1.0, "y": 1.0}],
        )
        poly = get_node_polygon(graph, "N1")
        assert poly == [(9.0, 4.0), (11.0, 4.0), (11.0, 6.0), (9.0, 6.0)]

    def test_no_verts_returns_empty_list(self):
        graph = nx.DiGraph()
        graph.add_node("N1", x=0.0, y=0.0, verts=[])
        assert get_node_polygon(graph, "N1") == []


# ---------------------------------------------------------------------------
# Graph analysis
# ---------------------------------------------------------------------------

class TestFindOrphanedNodes:
    def test_simple_map_has_one_orphan(self):
        result = analyse_map(SIMPLE_MAP)
        assert result.orphaned_nodes == ["WP1"]

    def test_mixed_actions_map_has_no_orphans(self):
        result = analyse_map(MIXED_ACTIONS_MAP)
        assert result.orphaned_nodes == []

    def test_isolated_node_is_orphaned(self):
        graph = nx.DiGraph()
        graph.add_node("A", x=0.0, y=0.0, verts=[])
        assert find_orphaned_nodes(graph) == ["A"]


class TestFindDisconnectedComponents:
    def test_fully_connected_map_is_single_component(self):
        result = analyse_map(MIXED_ACTIONS_MAP)
        assert len(result.disconnected_components) == 1

    def test_disconnected_graph_has_multiple_components(self):
        graph = nx.DiGraph()
        graph.add_node("A", x=0.0, y=0.0, verts=[])
        graph.add_node("B", x=1.0, y=0.0, verts=[])
        components = find_disconnected_components(graph)
        assert len(components) == 2


class TestIsBidirectionalEdge:
    def test_bidirectional_pair(self):
        graph = nx.DiGraph()
        graph.add_edge("A", "B")
        graph.add_edge("B", "A")
        assert is_bidirectional_edge(graph, "A", "B") is True

    def test_unidirectional_pair(self):
        graph = nx.DiGraph()
        graph.add_edge("A", "B")
        assert is_bidirectional_edge(graph, "A", "B") is False


class TestComputeStatistics:
    def test_mixed_actions_map_statistics(self):
        result = analyse_map(MIXED_ACTIONS_MAP)
        stats = result.statistics
        assert stats["num_nodes"] == 6
        assert stats["num_edges"] == 10
        assert stats["bidirectional_edge_count"] == 5
        assert stats["unidirectional_edge_count"] == 0
        assert stats["action_counts"] == {
            "navigate_to_pose": 4, "row_traversal": 4, "goal_align": 2,
        }
        assert stats["total_edge_length"] == pytest.approx(30.0)


class TestFindOverlappingInfluenceZones:
    def test_polygon_shapes_map_has_overlaps(self):
        result = analyse_map(POLYGON_SHAPES_MAP)
        pairs = {frozenset((o["node_a"], o["node_b"])) for o in result.overlaps}
        assert frozenset({"TriangleNode", "LargeNode"}) in pairs
        assert frozenset({"PentagonNode", "LargeNode"}) in pairs
        assert frozenset({"LargeNode", "RectangleNode"}) in pairs

    def test_simple_map_has_no_overlaps(self):
        result = analyse_map(SIMPLE_MAP)
        assert result.overlaps == []


# ---------------------------------------------------------------------------
# SVG generation
# ---------------------------------------------------------------------------

class TestGenerateSvg:
    def test_generates_valid_svg_file(self, tmp_path):
        from topological_navigation.networkx_utils import build_graph_from_tmap
        from topological_navigation.tmap_utils import load_tmap2_file
        graph = build_graph_from_tmap(load_tmap2_file(MIXED_ACTIONS_MAP))

        out = tmp_path / "map.svg"
        svg = generate_svg(graph, str(out), title="test")

        assert out.is_file()
        assert svg.startswith("<svg")
        assert svg.strip().endswith("</svg>")
        # Bidirectional edges must not reference an arrow marker.
        assert 'marker-end="url(#arrow-2ca02c)"' not in svg.split("<defs>")[0]

    def test_directional_edges_get_arrow_marker(self, tmp_path):
        from topological_navigation.networkx_utils import build_graph_from_tmap
        from topological_navigation.tmap_utils import load_tmap2_file
        graph = build_graph_from_tmap(load_tmap2_file(SIMPLE_MAP))

        out = tmp_path / "simple.svg"
        svg = generate_svg(graph, str(out))
        assert 'marker-end="url(#arrow-' in svg

    def test_raises_on_empty_graph(self, tmp_path):
        graph = nx.DiGraph()
        with pytest.raises(ValueError):
            generate_svg(graph, str(tmp_path / "empty.svg"))

    def test_output_is_well_formed_xml(self, tmp_path):
        """The generated document must always be parseable XML."""
        import xml.etree.ElementTree as ET
        from topological_navigation.networkx_utils import build_graph_from_tmap
        from topological_navigation.tmap_utils import load_tmap2_file
        graph = build_graph_from_tmap(load_tmap2_file(COMPLEX_MAP))

        out = tmp_path / "complex.svg"
        svg = generate_svg(graph, str(out), title="A & B <weird> \"title\"")
        ET.fromstring(svg)  # raises ParseError if malformed

    def test_special_characters_in_names_are_escaped(self, tmp_path):
        """Node/action/edge names with XML metacharacters must not break the SVG."""
        import xml.etree.ElementTree as ET

        graph = nx.DiGraph()
        graph.add_node("N<1>", x=0.0, y=0.0, verts=[])
        graph.add_node('N&"2', x=5.0, y=0.0, verts=[])
        graph.add_edge(
            "N<1>", 'N&"2',
            edge_id="e<1>&'2\"", action="go & <stop>",
            action_type="", properties={}, weight=1.0,
        )

        out = tmp_path / "special.svg"
        svg = generate_svg(graph, str(out), title="Map <1> & \"2\"")

        # Must parse as valid XML despite the special characters.
        ET.fromstring(svg)
        # Raw metacharacters must not appear unescaped in text content.
        assert "<1>" not in svg
        assert "go & <stop>" not in svg
        assert "&lt;1&gt;" in svg

    def test_non_finite_node_coordinates_do_not_break_svg(self, tmp_path):
        """NaN/Inf coordinates must be sanitised, not leak into the markup."""
        import re
        import xml.etree.ElementTree as ET

        graph = nx.DiGraph()
        graph.add_node("A", x=float("nan"), y=float("inf"), verts=[])
        graph.add_node("B", x=1.0, y=1.0, verts=[])
        graph.add_edge(
            "A", "B", edge_id="A_B", action="navigate_to_pose",
            action_type="", properties={}, weight=1.0,
        )

        out = tmp_path / "nonfinite.svg"
        svg = generate_svg(graph, str(out))

        ET.fromstring(svg)
        # Every numeric-looking attribute value must be a finite number;
        # a loose whole-document substring search would false-positive on
        # unrelated text (e.g. font-family names), so inspect attribute
        # values directly instead.
        for value in re.findall(r'"(-?\d[\d.eE+-]*)"', svg):
            assert value.lower() not in ("nan", "inf", "-inf", "infinity", "-infinity")
            assert math.isfinite(float(value))

    def test_degenerate_width_height_do_not_break_svg(self, tmp_path):
        """A width/height smaller than 2x margin must not produce an invalid document."""
        import xml.etree.ElementTree as ET
        from topological_navigation.networkx_utils import build_graph_from_tmap
        from topological_navigation.tmap_utils import load_tmap2_file
        graph = build_graph_from_tmap(load_tmap2_file(SIMPLE_MAP))

        out = tmp_path / "tiny.svg"
        svg = generate_svg(graph, str(out), width=10, height=10, margin=40.0)

        ET.fromstring(svg)

    def test_all_nodes_at_same_position_do_not_break_svg(self, tmp_path):
        """A single point (zero span) map must still render valid SVG."""
        import xml.etree.ElementTree as ET

        graph = nx.DiGraph()
        graph.add_node("A", x=3.0, y=3.0, verts=[])
        graph.add_node("B", x=3.0, y=3.0, verts=[])

        out = tmp_path / "single_point.svg"
        svg = generate_svg(graph, str(out))

        ET.fromstring(svg)


# ---------------------------------------------------------------------------
# analyse_map / AnalysisResult
# ---------------------------------------------------------------------------

class TestAnalyseMap:
    def test_valid_map_reports_valid(self):
        result = analyse_map(MIXED_ACTIONS_MAP)
        assert isinstance(result, AnalysisResult)
        assert result.is_valid is True

    def test_map_with_orphan_is_invalid(self):
        result = analyse_map(SIMPLE_MAP)
        assert result.is_valid is False

    def test_map_with_overlap_is_invalid(self):
        result = analyse_map(POLYGON_SHAPES_MAP)
        assert result.is_valid is False

    def test_disconnected_submaps_alone_do_not_invalidate(self):
        # complex_map has disconnected submaps AND orphans (NoGoZone /
        # TopicLocaliseNode are isolated nodes), so build a graph by hand
        # to isolate the "disconnected-only" scenario.
        graph = nx.DiGraph()
        graph.add_node("A", x=0.0, y=0.0, verts=[])
        graph.add_node("B", x=0.0, y=0.0, verts=[])
        graph.add_edge("A", "B", action="navigate_to_pose", action_type="", properties={}, weight=1.0)
        graph.add_node("C", x=100.0, y=100.0, verts=[])
        graph.add_node("D", x=100.0, y=101.0, verts=[])
        graph.add_edge("C", "D", action="navigate_to_pose", action_type="", properties={}, weight=1.0)

        components = find_disconnected_components(graph)
        orphans = find_orphaned_nodes(graph)
        overlaps = find_overlapping_influence_zones(graph)

        assert len(components) == 2
        assert orphans == ["A", "C"]  # sources of each sub-map have no incoming edges... but that's expected
        # Sanity: components alone shouldn't be conflated with validity;
        # validity is computed from schema + orphans + overlaps only.
        result = AnalysisResult(
            map_file="dummy",
            schema_valid=True,
            schema_message="ok",
            orphaned_nodes=[],
            disconnected_components=components,
            statistics=compute_statistics(graph),
            overlaps=overlaps,
        )
        assert result.is_valid is True

    def test_generates_svg_when_path_given(self, tmp_path):
        out = tmp_path / "out.svg"
        result = analyse_map(MIXED_ACTIONS_MAP, svg_path=str(out))
        assert result.svg_path == str(out)
        assert out.is_file()

    def test_format_report_contains_all_sections(self):
        result = analyse_map(MIXED_ACTIONS_MAP)
        report = result.format_report()
        for section in (
            "[Schema validation]", "[Orphaned nodes]",
            "[Disconnected sub-maps]", "[Overlapping influence zones]",
            "[Statistics]",
        ):
            assert section in report


# ---------------------------------------------------------------------------
# Minification
# ---------------------------------------------------------------------------

class TestMinifyMap:
    def test_default_anchors_produce_smaller_valid_roundtrip(self, tmp_path):
        out = tmp_path / "out.min.yaml"
        result = minify_map(COMPLEX_MAP, output_file=str(out))

        assert isinstance(result, MinifyResult)
        assert out.is_file()
        assert result.minified_size <= result.original_size
        assert result.schema_valid is True

        with open(out, encoding="utf-8") as fh:
            reparsed = yaml.load(fh, Loader=CustomSafeLoader)
        reparsed_body = {k: v for k, v in reparsed.items() if k != "anchors"}
        assert reparsed_body == load_tmap2_file(COMPLEX_MAP)

    def test_low_thresholds_create_anchors(self, tmp_path):
        out = tmp_path / "out.min.yaml"
        result = minify_map(
            COMPLEX_MAP, output_file=str(out), min_size=1, min_occurrences=2
        )
        assert result.anchors_created > 0
        assert result.occurrences_collapsed >= result.anchors_created * 2

        with open(out, encoding="utf-8") as fh:
            text = fh.read()
        assert "anchors:" in text
        assert "&" in text  # at least one anchor definition
        assert "*" in text  # at least one alias reference

    def test_no_anchors_disables_anchor_section(self, tmp_path):
        out = tmp_path / "out.min.yaml"
        result = minify_map(
            COMPLEX_MAP, output_file=str(out), anchors=False, min_size=1, min_occurrences=2
        )
        assert result.anchors_created == 0
        assert result.occurrences_collapsed == 0

        with open(out, encoding="utf-8") as fh:
            text = fh.read()
        assert "anchors:" not in text

    def test_strip_comments_removes_leading_banner(self, tmp_path):
        out_kept = tmp_path / "kept.min.yaml"
        out_stripped = tmp_path / "stripped.min.yaml"
        minify_map(COMPLEX_MAP, output_file=str(out_kept), strip_comments=False)
        minify_map(COMPLEX_MAP, output_file=str(out_stripped), strip_comments=True)

        with open(COMPLEX_MAP, encoding="utf-8") as fh:
            original_first_line = fh.readline()

        if original_first_line.lstrip().startswith("#"):
            assert out_kept.read_text().startswith("#")
            assert not out_stripped.read_text().startswith("#")

    def test_strip_unreachable_removes_isolated_nodes_and_stays_valid(self, tmp_path):
        graph_before = load_tmap2_file(COMPLEX_MAP)
        node_names = [n["node"]["name"] for n in graph_before["nodes"]]
        start = node_names[0]

        out = tmp_path / "out.min.yaml"
        result = minify_map(COMPLEX_MAP, output_file=str(out), strip_unreachable_from=start)

        assert result.stripped_nodes >= 0
        analysis = analyse_map(str(out))
        assert analysis.orphaned_nodes == [] or analysis.orphaned_nodes == [start]
        assert len(analysis.disconnected_components) <= 1

    def test_strip_unreachable_raises_for_unknown_node(self, tmp_path):
        out = tmp_path / "out.min.yaml"
        with pytest.raises(ValueError):
            minify_map(COMPLEX_MAP, output_file=str(out), strip_unreachable_from="NoSuchNode")

    def test_derives_output_path_when_not_given(self, tmp_path):
        src = tmp_path / "my_map.tmap2.yaml"
        src.write_text(open(COMPLEX_MAP, encoding="utf-8").read())
        result = minify_map(str(src))
        expected = tmp_path / "my_map.min.tmap2.yaml"
        assert result.output_file == str(expected)
        assert expected.is_file()

    def test_format_report_contains_key_figures(self, tmp_path):
        out = tmp_path / "out.min.yaml"
        result = minify_map(COMPLEX_MAP, output_file=str(out), min_size=1, min_occurrences=2)
        report = result.format_report()
        assert "Original size" in report
        assert "Minified size" in report
        assert "Reduction" in report
        assert "Anchors created" in report

    def test_minifying_an_already_minified_file_is_idempotent(self, tmp_path):
        pass1 = tmp_path / "pass1.min.yaml"
        pass2 = tmp_path / "pass2.min.yaml"
        minify_map(COMPLEX_MAP, output_file=str(pass1), min_size=1, min_occurrences=2)
        assert "anchors:" in pass1.read_text()

        result2 = minify_map(str(pass1), output_file=str(pass2), min_size=1, min_occurrences=2)
        assert result2.schema_valid is True
        assert pass2.read_text() == pass1.read_text()


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

class TestCli:
    def test_check_exits_zero_for_valid_map(self, capsys):
        with pytest.raises(SystemExit) as exc_info:
            main(["check", MIXED_ACTIONS_MAP])
        assert exc_info.value.code == 0

    def test_check_exits_one_for_invalid_map(self, capsys):
        with pytest.raises(SystemExit) as exc_info:
            main(["check", SIMPLE_MAP])
        assert exc_info.value.code == 1

    def test_check_exits_two_for_missing_file(self, capsys):
        with pytest.raises(SystemExit) as exc_info:
            main(["check", "/no/such/file.yaml"])
        assert exc_info.value.code == 2

    def test_svg_command_writes_file(self, tmp_path, capsys):
        out = tmp_path / "cli_out.svg"
        main(["svg", MIXED_ACTIONS_MAP, "-o", str(out)])
        assert out.is_file()

    def test_analyse_command_prints_report(self, capsys):
        main(["analyse", MIXED_ACTIONS_MAP])
        captured = capsys.readouterr()
        assert "Map analysis report" in captured.out

    def test_build_arg_parser_requires_subcommand(self):
        parser = build_arg_parser()
        with pytest.raises(SystemExit):
            parser.parse_args([])

    def test_minify_command_writes_file_and_prints_report(self, tmp_path, capsys):
        out = tmp_path / "cli_out.min.yaml"
        main(["minify", COMPLEX_MAP, "-o", str(out)])
        assert out.is_file()
        captured = capsys.readouterr()
        assert "Minify report" in captured.out

    def test_minify_command_exits_two_for_missing_file(self, capsys):
        with pytest.raises(SystemExit) as exc_info:
            main(["minify", "/no/such/file.yaml"])
        assert exc_info.value.code == 2

    def test_minify_command_exits_two_for_bad_strip_unreachable(self, tmp_path, capsys):
        out = tmp_path / "cli_out.min.yaml"
        with pytest.raises(SystemExit) as exc_info:
            main(["minify", COMPLEX_MAP, "-o", str(out), "--strip-unreachable", "NoSuchNode"])
        assert exc_info.value.code == 2
