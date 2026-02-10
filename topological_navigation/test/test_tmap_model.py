import pytest
import os
import json
import tempfile
from topological_navigation.tmap_model import TopologicalMapModel, MapValidationError, NodeNotFoundError, DuplicateError

# Mock schema path
SCHEMA_PATH = os.path.join(os.path.dirname(__file__), '..', 'config', 'tmap-schema.yaml')

@pytest.fixture
def empty_model():
    return TopologicalMapModel(schema_path=SCHEMA_PATH)

@pytest.fixture
def simple_map_data():
    return {
        "meta": {"last_updated": "2023-01-01_00-00-00"},
        "metric_map": "test_map",
        "name": "test_topo_map",
        "pointset": "test_pointset",
        "transformation": {
            "rotation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
            "translation": {"x": 0.0, "y": 0.0, "z": 0.0},
            "child": "topo_map",
            "parent": "map"
        },
        "nodes": []
    }

def test_initialization():
    model = TopologicalMapModel(schema_path=SCHEMA_PATH)
    assert model.tmap is not None
    assert "nodes" in model.tmap
    assert "meta" in model.tmap
    assert model.tmap["nodes"] == []
    assert model.schema is not None

def test_add_node(empty_model, simple_map_data):
    empty_model.tmap = simple_map_data
    
    pose = {"position": {"x": 1.0, "y": 2.0, "z": 0.0}, "orientation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}}
    empty_model.add_node("WayPoint1", pose)
    
    node = empty_model.get_node("WayPoint1")
    assert node is not None
    assert node["node"]["name"] == "WayPoint1"
    assert node["node"]["pose"]["position"]["x"] == 1.0

def test_add_duplicate_node(empty_model, simple_map_data):
    empty_model.tmap = simple_map_data
    pose = {"position": {"x": 1.0, "y": 2.0, "z": 0.0}, "orientation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}}
    
    empty_model.add_node("WayPoint1", pose)
    
    with pytest.raises(DuplicateError):
        empty_model.add_node("WayPoint1", pose)

def test_add_edge(empty_model, simple_map_data):
    empty_model.tmap = simple_map_data
    pose = {"position": {"x": 0.0, "y": 0.0, "z": 0.0}, "orientation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}}
    
    empty_model.add_node("WP1", pose)
    empty_model.add_node("WP2", pose)
    
    empty_model.add_edge("WP1", "WP2", "move_base")
    
    node1 = empty_model.get_node("WP1")
    assert len(node1["node"]["edges"]) == 1
    assert node1["node"]["edges"][0]["node"] == "WP2"
    assert node1["node"]["edges"][0]["action"] == "move_base"

def test_add_edge_missing_node(empty_model, simple_map_data):
    empty_model.tmap = simple_map_data
    pose = {"position": {"x": 0.0, "y": 0.0, "z": 0.0}, "orientation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}}
    empty_model.add_node("WP1", pose)
    
    with pytest.raises(NodeNotFoundError):
        empty_model.add_edge("WP1", "NonExistent", "move_base")

def test_validation_success(empty_model, simple_map_data):
    empty_model.tmap = simple_map_data
    pose = {"position": {"x": 0.0, "y": 0.0, "z": 0.0}, "orientation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}}
    empty_model.add_node("WP1", pose)
    
    # Validation should pass
    empty_model.validate()

def test_validation_failure(empty_model, simple_map_data):
    empty_model.tmap = simple_map_data
    # Corrupt the data structure
    del empty_model.tmap["meta"] 
    
    with pytest.raises(MapValidationError):
        empty_model.validate()

def test_save_load(empty_model, simple_map_data):
    empty_model.tmap = simple_map_data
    pose = {"position": {"x": 0.0, "y": 0.0, "z": 0.0}, "orientation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}}
    empty_model.add_node("WP1", pose)
    
    with tempfile.NamedTemporaryFile(suffix=".tmap2", mode='w+', delete=False) as tmp:
        tmp_path = tmp.name
    
    try:
        empty_model.save(tmp_path)
        
        new_model = TopologicalMapModel(schema_path=SCHEMA_PATH)
        new_model.load(tmp_path)
        
        assert new_model.tmap["name"] == simple_map_data["name"]
        assert len(new_model.tmap["nodes"]) == 1
        assert new_model.get_node("WP1") is not None
        
    finally:
        os.remove(tmp_path)

