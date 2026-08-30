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
* **Minification** – writes a smaller, semantically-identical copy of
  the map by collapsing repeated data structures (e.g. shared
  `properties` or `verts` blocks) into named YAML anchors, and
  optionally stripping nodes/edges unreachable from a given node.

## Usage

```bash
# Full human-readable analysis report
ros2 run topological_navigation map_analyser.py analyse my_map.tmap2.yaml

# Full report + SVG rendering
ros2 run topological_navigation map_analyser.py analyse my_map.tmap2.yaml --svg my_map.svg

# CI-friendly validity check (exit code reflects the result)
ros2 run topological_navigation map_analyser.py check my_map.tmap2.yaml

# Treat disconnected sub-maps as an error, and skip the influence-zone check
ros2 run topological_navigation map_analyser.py check my_map.tmap2.yaml \
  --sub-map-separation=error --influence-zone-overlap=false

# SVG rendering only
ros2 run topological_navigation map_analyser.py svg my_map.tmap2.yaml -o my_map.svg

# Minify (writes my_map.min.tmap2.yaml alongside the input by default)
ros2 run topological_navigation map_analyser.py minify my_map.tmap2.yaml
```

The script can also be run directly without a ROS 2 environment:

```bash
python3 topological_navigation/topological_navigation/map_analyser.py check my_map.tmap2.yaml
```

## Configuring check severities

Each of the four checks can be independently turned off, or have its
severity set to `warning` or `error`, via a `--<check>={false,warning,error}`
switch on the `analyse`/`check` subcommands:

| Switch | Check | Default severity |
|--------|-------|-------------------|
| `--schema-check` | Schema compliance | `error` |
| `--orphaned-node` | Orphaned nodes | `error` |
| `--sub-map-separation` | Disconnected sub-maps | `warning` |
| `--influence-zone-overlap` | Overlapping influence zones | `warning` |

* `false` (also accepted: `off`, `disable`, `disabled`, `none`) disables
  the check entirely: it is neither run nor printed in the report.
* `warning` runs the check and prints its result, but a failure does not
  affect the `check` command's exit code.
* `error` (also accepted: `true`, `on`) runs the check and a failure
  causes `check` to exit with code `1`.

Example: fail CI only on schema errors and orphaned nodes, ignore
disconnected sub-maps and influence zone overlaps entirely:

```bash
map_analyser.py check my_map.tmap2.yaml \
  --sub-map-separation=false --influence-zone-overlap=false
```

## `minify` command

Topological maps often contain many repeated data structures (e.g. the
same `properties` dict or influence-zone `verts` polygon reused across
dozens of nodes/edges). `minify` rewrites the map into an equivalent but
smaller file by finding these repeated subtrees and replacing them with
YAML anchors/aliases, collected under a top-level `anchors:` key.

```bash
map_analyser.py minify my_map.tmap2.yaml
map_analyser.py minify my_map.tmap2.yaml -o compact.tmap2.yaml
map_analyser.py minify my_map.tmap2.yaml --strip-unreachable Charging
```

If `-o`/`--output` is omitted, the output path is derived from the
input file by inserting `.min` before the extension, e.g.
`my_map.tmap2.yaml` -> `my_map.min.tmap2.yaml`.

Anchor names are chosen from the map's schema/content (e.g.
`properties_row_entry`, `verts_2x2`) rather than being generic
auto-generated ids, so the minified file stays human-readable.

| Switch | Meaning | Default |
|--------|---------|---------|
| `--anchors` / `--no-anchors` | Collapse repeated subtrees into named anchors | enabled |
| `--strip-comments` | Drop the file's leading comment block | disabled (comments kept) |
| `--flowstyle` | Emit compact flow-style YAML instead of block style | disabled |
| `--min-size N` | Minimum serialised size (chars) of a subtree to qualify for anchoring | `100` |
| `--min-occurrences N` | Minimum number of repeats required for a subtree to qualify for anchoring | `5` |
| `--strip-unreachable NODE` | Also drop nodes/edges not reachable via a directed path from `NODE` | disabled |

Notes:

* Only the leading top-of-file comment block is preserved; inline/
  per-node comments are not (the tool re-serialises the map from its
  parsed data structure, like the rest of this codebase).
* `--strip-unreachable NODE` computes the directed descendants of
  `NODE` (plus `NODE` itself) and keeps only those. This guarantees
  the result has no orphaned nodes (other than possibly `NODE` itself,
  if nothing points back to it) and is a single connected sub-map, so
  it always passes the `orphaned-node` and `sub-map-separation` checks.
* The tool re-parses its own output and schema-validates the written file, to catch any minification bug; it logs a summary of the size savings achieved.

## `check` command exit codes

The `check` command is designed for use in a GitHub Actions workflow:

| Exit code | Meaning |
|-----------|---------|
| `0` | Map is valid |
| `1` | Map is invalid |
| `2` | Map file not found, or another error occurred while loading it |

With the default severities, a map is considered **valid** if:

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
