"""Tests for generating occupancy images from topological maps."""

import yaml

from topological_navigation.topomap_to_map_image import (
    geometry_from_tmap,
    rasterize_geometry,
    write_map_yaml,
)


def _simple_tmap():
    return {
        "transformation": {
            "parent": "map",
            "topo_frame_id": "topo",
            "translation": {"x": 1.0, "y": 2.0, "z": 0.0},
            "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        },
        "nodes": [
            {
                "meta": {"node": "A"},
                "node": {
                    "name": "A",
                    "pose": {
                        "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                    },
                    "verts": [
                        {"x": -0.5, "y": -0.5},
                        {"x": 0.5, "y": -0.5},
                        {"x": 0.5, "y": 0.5},
                        {"x": -0.5, "y": 0.5},
                    ],
                    "edges": [{"edge_id": "A_B", "node": "B", "action": "go"}],
                },
            },
            {
                "meta": {"node": "B"},
                "node": {
                    "name": "B",
                    "pose": {
                        "position": {"x": 10.0, "y": 0.0, "z": 0.0},
                        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                    },
                    "edges": [],
                },
            },
        ],
    }


def test_geometry_extracts_edges_and_applies_transform():
    geometry = geometry_from_tmap(_simple_tmap())

    assert geometry.frame_id == "map"
    assert geometry.nodes["A"].point == (1.0, 2.0)
    assert geometry.nodes["B"].point == (11.0, 2.0)
    assert geometry.edges == (("A", "B"),)


def test_rasterize_whitens_edge_corridor_and_keeps_background_black():
    geometry = geometry_from_tmap(_simple_tmap(), apply_transform=False)
    raster = rasterize_geometry(
        geometry,
        white_extension_m=1.0,
        resolution=1.0,
        padding_m=1.0,
        include_node_polygons=False,
    )

    image = raster.image
    origin_x, origin_y = raster.origin
    top_y = origin_y + ((image.height - 1) * raster.resolution)

    def pixel_at(world_x, world_y):
        px = int(round((world_x - origin_x) / raster.resolution))
        py = int(round((top_y - world_y) / raster.resolution))
        return image.getpixel((px, py))

    assert pixel_at(5.0, 0.0) == 255
    assert pixel_at(5.0, 1.0) == 255
    assert image.getpixel((0, 0)) == 0


def test_write_map_yaml_uses_relative_image_path(tmp_path):
    geometry = geometry_from_tmap(_simple_tmap(), apply_transform=False)
    raster = rasterize_geometry(
        geometry,
        white_extension_m=1.0,
        resolution=0.5,
        padding_m=0.0,
    )
    image_path = tmp_path / "generated.png"
    yaml_path = tmp_path / "generated.yaml"
    raster.image.save(image_path)

    write_map_yaml(str(yaml_path), str(image_path), raster)

    with open(yaml_path, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)

    assert data["image"] == "generated.png"
    assert data["resolution"] == 0.5
    assert data["negate"] == 0
    assert data["occupied_thresh"] == 0.65
    assert data["free_thresh"] == 0.196
