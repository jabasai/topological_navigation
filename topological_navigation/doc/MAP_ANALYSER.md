# Topological Map Analyser

`map_analyser.py` is a stand-alone command-line tool for analysing
`.tmap2.yaml` topological map files. It requires no ROS 2 runtime and
only depends on `pyyaml`, `jsonschema` and `networkx`, so it can be run
locally or in a CI pipeline (e.g. a GitHub Actions workflow).

## What it checks

* **Schema compliance** – validates the map against
  `config/tmap-schema.yaml` (delegates to `validate_map.py`).
* **Orphaned nodes** – nodes with no incoming edges, i.e. nodes that
  can never be reached by navigating the graph.
* **Disconnected sub-maps** – weakly connected components of the
  topological graph. More than one component means the map contains
  islands of nodes that cannot reach each other.
* **Map statistics** – node/edge counts, total and average edge
  length, a breakdown of edge counts per action, and the number of
  bidirectional vs. unidirectional edges.
* **Overlapping influence zones** – pairs of nodes whose influence
  zone polygons (`verts`) overlap, including the case where a node's
  position falls inside another node's polygon.
* **SVG rendering** – a full-map SVG image with nodes as circles,
  edges colour-coded by action, bidirectional edges drawn as plain
  lines (no arrow head), and unidirectional edges drawn with an arrow
  head indicating direction of travel.

## Usage

```bash
# Full human-readable analysis report
ros2 run topological_navigation map_analyser.py analyse my_map.tmap2.yaml

# Full report + SVG rendering
ros2 run topological_navigation map_analyser.py analyse my_map.tmap2.yaml --svg my_map.svg

# CI-friendly validity check (exit code reflects the result)
ros2 run topological_navigation map_analyser.py check my_map.tmap2.yaml

# SVG rendering only
ros2 run topological_navigation map_analyser.py svg my_map.tmap2.yaml -o my_map.svg
```

The script can also be run directly without a ROS 2 environment:

```bash
python3 topological_navigation/topological_navigation/map_analyser.py check my_map.tmap2.yaml
```

## `check` command exit codes

The `check` command is designed for use in a GitHub Actions workflow:

| Exit code | Meaning |
|-----------|---------|
| `0` | Map is valid |
| `1` | Map is invalid |
| `2` | Map file not found, or another error occurred while loading it |

A map is considered **valid** if:

1. It is compliant with the JSON schema.
2. It has no orphaned nodes (nodes unreachable via any incoming edge).
3. No two influence zones overlap, and no node's position falls
   inside another node's influence zone.

Disconnected sub-maps are **not** treated as a validity failure: they
are printed as a warning in the report, since it is often expected for
a map to contain isolated auxiliary nodes (e.g. no-go zones, topic
localisation markers) that are not part of the main navigable graph.

## Example GitHub workflow

See `.github/workflows/map-analyser.yaml` for a workflow that runs
`map_analyser.py check` against every `.tmap2.yaml`/fixture map in the
repository and uploads the generated SVG files as a workflow artefact.
