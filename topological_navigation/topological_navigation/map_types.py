#!/usr/bin/env python3
"""
Typed data structures for topological maps.
Provides type-safe interfaces for nodes, edges, and maps.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Any
import yaml
import json


# ===== YAML Loader =====

class CustomSafeLoader(yaml.SafeLoader):
    """
    Custom YAML loader that ensures poses and translations are float-type.
    ROS 2 messages (Vector3, Pose, etc.) have assertions for float-type [x,y,z,w] keys.
    """
    def construct_mapping(self, node, deep=False):
        mapping = super().construct_mapping(node, deep=deep)
        
        # Convert int to float for pose/vector keys
        for key in ['x', 'y', 'z', 'w', 'yaw_goal_tolerance', 'xy_goal_tolerance']:
            if key in mapping and isinstance(mapping[key], int):
                mapping[key] = float(mapping[key])
        
        return mapping


# ===== Data Classes =====

@dataclass
class TopologicalPose:
    """Represents a pose in topological map"""
    position: Dict[str, float]  # {x, y, z}
    orientation: Dict[str, float]  # {x, y, z, w}
    
    @staticmethod
    def from_dict(data: Dict) -> 'TopologicalPose':
        return TopologicalPose(
            position=data.get('position', {'x': 0.0, 'y': 0.0, 'z': 0.0}),
            orientation=data.get('orientation', {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0})
        )


@dataclass
class TopologicalEdge:
    """Represents an edge in topological map"""
    edge_id: str
    node: str  # Target node name
    action: str
    action_type: str
    properties: Dict[str, Any] = field(default_factory=dict)
    
    @staticmethod
    def from_dict(data: Dict) -> 'TopologicalEdge':
        return TopologicalEdge(
            edge_id=data['edge_id'],
            node=data['node'],
            action=data.get('action', ''),
            action_type=data.get('action_type', ''),
            properties=data.get('properties', {})
        )
    
    def to_dict(self) -> Dict:
        """Convert back to dictionary format"""
        result = {
            'edge_id': self.edge_id,
            'node': self.node,
            'action': self.action,
            'action_type': self.action_type
        }
        if self.properties:
            result['properties'] = self.properties
        return result


@dataclass
class TopologicalNode:
    """Represents a node in topological map"""
    name: str
    pose: TopologicalPose
    parent_frame: str = "map"
    verts: List[Dict[str, float]] = field(default_factory=list)  # Influence zone polygon
    properties: Dict[str, Any] = field(default_factory=dict)
    edges: List[TopologicalEdge] = field(default_factory=list)
    
    # Metadata
    map_name: str = ""
    pointset: str = ""
    
    @staticmethod
    def from_dict(data: Dict) -> 'TopologicalNode':
        """Create TopologicalNode from dictionary (tmap2 format)"""
        node_data = data.get('node', data)  # Handle both wrapped and unwrapped format
        meta_data = data.get('meta', {})
        
        return TopologicalNode(
            name=node_data['name'],
            pose=TopologicalPose.from_dict(node_data.get('pose', {})),
            parent_frame=node_data.get('parent_frame', 'map'),
            verts=node_data.get('verts', []),
            properties=node_data.get('properties', {}),
            edges=[TopologicalEdge.from_dict(e) for e in node_data.get('edges', [])],
            map_name=meta_data.get('map', ''),
            pointset=meta_data.get('pointset', '')
        )
    
    def to_dict(self) -> Dict:
        """Convert back to tmap2 dictionary format"""
        return {
            'meta': {
                'map': self.map_name,
                'node': self.name,
                'pointset': self.pointset
            },
            'node': {
                'name': self.name,
                'pose': {
                    'position': self.pose.position,
                    'orientation': self.pose.orientation
                },
                'parent_frame': self.parent_frame,
                'verts': self.verts,
                'properties': self.properties,
                'edges': [e.to_dict() for e in self.edges]
            }
        }
    
    def get_property(self, key: str, default: Any = None) -> Any:
        """Safe property access with default"""
        return self.properties.get(key, default)
    
    def get_edge(self, edge_id: str) -> Optional[TopologicalEdge]:
        """Get edge by ID"""
        for edge in self.edges:
            if edge.edge_id == edge_id:
                return edge
        return None


@dataclass
class TopologicalMap:
    """Represents a complete topological map"""
    name: str
    metric_map: str
    pointset: str
    nodes: List[TopologicalNode] = field(default_factory=list)
    transformation: Dict[str, Any] = field(default_factory=dict)
    
    @staticmethod
    def from_dict(data: Dict) -> 'TopologicalMap':
        """Create TopologicalMap from dictionary (tmap2 format)"""
        return TopologicalMap(
            name=data.get('name', ''),
            metric_map=data.get('metric_map', ''),
            pointset=data.get('pointset', ''),
            transformation=data.get('transformation', {}),
            nodes=[TopologicalNode.from_dict(n) for n in data.get('nodes', [])]
        )
    
    @staticmethod
    def from_yaml(filename: str) -> 'TopologicalMap':
        """Load topological map from YAML file"""
        with open(filename, 'r') as f:
            data = yaml.load(f, Loader=CustomSafeLoader)
        return TopologicalMap.from_dict(data)
    
    @staticmethod
    def from_json_string(json_str: str) -> 'TopologicalMap':
        """Load topological map from JSON string"""
        data = json.loads(json_str)
        return TopologicalMap.from_dict(data)
    
    def to_dict(self) -> Dict:
        """Convert back to tmap2 dictionary format"""
        return {
            'name': self.name,
            'metric_map': self.metric_map,
            'pointset': self.pointset,
            'transformation': self.transformation,
            'nodes': [n.to_dict() for n in self.nodes]
        }
    
    def to_json_string(self) -> str:
        """Convert to JSON string for publishing"""
        return json.dumps(self.to_dict())
    
    def get_node(self, node_name: str) -> Optional[TopologicalNode]:
        """Get node by name"""
        for node in self.nodes:
            if node.name == node_name:
                return node
        return None
    
    def get_node_names(self) -> List[str]:
        """Get list of all node names"""
        return [n.name for n in self.nodes]
    
    def get_edge(self, edge_id: str) -> Optional[TopologicalEdge]:
        """Get edge by ID from any node"""
        for node in self.nodes:
            edge = node.get_edge(edge_id)
            if edge:
                return edge
        return None


# ===== Property Accessor Helpers =====

class PropertyAccessor:
    """Helper class for safe property access"""
    
    @staticmethod
    def get_node_property(node: Dict, key: str, default: Any = None) -> Any:
        """
        Get property from node dictionary (legacy format).
        Usage: PropertyAccessor.get_node_property(node_dict, 'xy_goal_tolerance', 0.5)
        """
        return node.get("node", {}).get("properties", {}).get(key, default)
    
    @staticmethod
    def get_edge_property(edge: Dict, key: str, default: Any = None) -> Any:
        """
        Get property from edge dictionary (legacy format).
        Usage: PropertyAccessor.get_edge_property(edge_dict, 'max_speed', 0.8)
        """
        return edge.get("properties", {}).get(key, default)
    
    @staticmethod
    def get_nested_property(props: Dict, *keys, default: Any = None) -> Any:
        """
        Get nested property with dotted key access.
        Usage: PropertyAccessor.get_nested_property(props, 'roboflow', 'enabled', default=False)
        """
        value = props
        for key in keys:
            if isinstance(value, dict):
                value = value.get(key)
                if value is None:
                    return default
            else:
                return default
        return value if value is not None else default


# ===== Backward Compatibility Helpers =====

def node_to_legacy_dict(node: TopologicalNode) -> Dict:
    """Convert TopologicalNode to legacy dict format for compatibility"""
    return node.to_dict()


def edge_to_legacy_dict(edge: TopologicalEdge) -> Dict:
    """Convert TopologicalEdge to legacy dict format for compatibility"""
    return edge.to_dict()


def map_to_legacy_dict(tmap: TopologicalMap) -> Dict:
    """Convert TopologicalMap to legacy dict format for compatibility"""
    return tmap.to_dict()
