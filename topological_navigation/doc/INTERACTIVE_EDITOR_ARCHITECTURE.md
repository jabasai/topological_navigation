# Interactive Map Editor Architecture

## System Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                    Interactive Map Editor                        │
│                  (interactive_map_editor.py)                     │
└─────────────────────────────────────────────────────────────────┘
                              │
                              │ loads/saves
                              ▼
                    ┌──────────────────┐
                    │  YAML Map File   │
                    │  (.tmap2.yaml)   │
                    └──────────────────┘
                              │
                              │ parsed into
                              ▼
                    ┌──────────────────┐
                    │  Internal Map    │
                    │  Data Structure  │
                    │  (Python dict)   │
                    └──────────────────┘
                              │
                              │ creates
                              ▼
                    ┌──────────────────┐
                    │ Interactive      │
                    │ Marker Server    │
                    └──────────────────┘
                              │
                              │ publishes to
                              ▼
                    ┌──────────────────┐
                    │  RViz2 Display   │
                    │  (Interactive    │
                    │   Markers)       │
                    └──────────────────┘
                              │
                              │ user interaction
                              ▼
                    ┌──────────────────┐
                    │  Feedback        │
                    │  (pose updates)  │
                    └──────────────────┘
                              │
                              │ updates
                              ▼
                    ┌──────────────────┐
                    │  Internal Map    │
                    │  Data Structure  │
                    └──────────────────┘
                              │
                              │ on save
                              ▼
                    ┌──────────────────┐
                    │  YAML Map File   │
                    │  (updated)       │
                    └──────────────────┘
```

## Component Details

### 1. Interactive Map Editor Node

**Class**: `InteractiveMapEditor(Node)`

**Responsibilities**:
- Load topological map from YAML
- Create interactive markers for each node
- Handle user interactions (drag/rotate)
- Update internal map data
- Save changes to YAML
- Publish updated map

**Key Methods**:
```python
load_map()                    # Load YAML → dict
save_map()                    # dict → YAML (with backup)
create_interactive_markers()  # Create markers for all nodes
create_node_marker(node)      # Create 6-DOF marker for one node
process_feedback(feedback)    # Handle marker movement/rotation
publish_map()                 # Publish to /topological_map_2
```

### 2. Interactive Marker Server

**Purpose**: ROS2 service that manages interactive markers

**Topics**:
- `/topological_map_editor/update` - Marker updates
- `/topological_map_editor/feedback` - User interactions

**Marker Types**:
- Sphere (node visualization)
- Arrow (orientation indicator)
- Text (node name label)
- Control rings (6-DOF manipulation)

### 3. RViz2 Display

**Display Type**: InteractiveMarkers

**User Interactions**:
- Left-click + drag: Move marker
- Control rings: Axis-specific movement/rotation
- Right-click: Context menu (if available)

**Visual Elements**:
- Blue sphere: Node position
- Red arrow: Node orientation
- White text: Node name
- Colored rings: Control axes (red=X, green=Y, blue=Z)

## Data Flow

### Loading Phase

```
1. User launches node with map_file parameter
   ↓
2. load_map() reads YAML file
   ↓
3. YAML parsed into Python dict (self.tmap)
   ↓
4. create_interactive_markers() iterates over nodes
   ↓
5. For each node: create_node_marker() creates marker
   ↓
6. Markers registered with InteractiveMarkerServer
   ↓
7. server.applyChanges() publishes to RViz2
```

### Interaction Phase

```
1. User drags marker in RViz2
   ↓
2. RViz2 sends feedback to InteractiveMarkerServer
   ↓
3. process_feedback() callback invoked
   ↓
4. Feedback contains: marker_name, new pose, event_type
   ↓
5. Find node in self.tmap by name
   ↓
6. Update node['pose']['position'] and ['orientation']
   ↓
7. Log update to console
   ↓
8. server.applyChanges() updates RViz2 display
```

### Saving Phase

```
1. User calls /interactive_map_editor/save_map service
   ↓
2. save_map() method invoked
   ↓
3. Backup original file: map.yaml → map.yaml.backup
   ↓
4. Write self.tmap to YAML file
   ↓
5. If save fails: restore backup
   ↓
6. publish_map() sends to /topological_map_2
   ↓
7. Service returns success response
```

## Interactive Marker Structure

### Node Marker Composition

```
InteractiveMarker (node_name)
├── Header
│   └── frame_id: "map" (or node's parent_frame)
├── Pose
│   ├── position: {x, y, z}
│   └── orientation: {x, y, z, w}
├── Controls (8 total)
│   ├── [0] Main Control (MOVE_ROTATE_3D)
│   │   ├── Sphere marker (blue, 0.5m)
│   │   └── Always visible
│   ├── [1] Move X (MOVE_AXIS, red)
│   ├── [2] Move Y (MOVE_AXIS, green)
│   ├── [3] Move Z (MOVE_AXIS, blue)
│   ├── [4] Rotate X (ROTATE_AXIS, red)
│   ├── [5] Rotate Y (ROTATE_AXIS, green)
│   ├── [6] Rotate Z (ROTATE_AXIS, blue)
│   ├── [7] Orientation Arrow (red, 0.75m)
│   └── [8] Text Label (white, node name)
└── Feedback Callback: process_feedback()
```

### Control Orientations

Each control has a specific orientation to define its axis:

```python
# Move/Rotate X (red)
orientation: {w: 1.0, x: 1.0, y: 0.0, z: 0.0}

# Move/Rotate Y (green)
orientation: {w: 1.0, x: 0.0, y: 1.0, z: 0.0}

# Move/Rotate Z (blue)
orientation: {w: 1.0, x: 0.0, y: 0.0, z: 1.0}
```

## ROS2 Interface

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| map_file | string | (required) | Path to .tmap2.yaml file |
| auto_save | bool | false | Auto-save every 30s |
| marker_scale | float | 0.5 | Marker size in meters |

### Topics

#### Published
- `/topological_map_2` (std_msgs/String)
  - Updated map in YAML format
  - Published after save
  - Latching QoS (TRANSIENT_LOCAL)

#### Subscribed
- None (uses Interactive Marker Server)

### Services

- `/interactive_map_editor/save_map` (std_srvs/Trigger)
  - Request: (empty)
  - Response: {success: bool, message: string}

### Action Servers
- None

### Action Clients
- None

## Error Handling

### Load Errors

```python
try:
    with open(self.map_file, 'r') as f:
        self.tmap = yaml.safe_load(f)
except Exception as e:
    self.get_logger().error(f'Failed to load map: {e}')
    sys.exit(1)  # Fatal error, cannot continue
```

### Save Errors

```python
try:
    # Backup original
    os.rename(self.map_file, backup_file)
    
    # Save updated map
    with open(self.map_file, 'w') as f:
        yaml.dump(self.tmap, f)
        
except Exception as e:
    self.get_logger().error(f'Failed to save map: {e}')
    
    # Restore backup
    if os.path.exists(backup_file):
        os.rename(backup_file, self.map_file)
```

## Performance Considerations

### Marker Count
- One marker per node
- Test map: 5 nodes = 5 markers
- Large map: 100 nodes = 100 markers
- RViz2 handles 100+ markers efficiently

### Update Frequency
- Feedback processed on every marker movement
- Updates are immediate (no batching)
- Console logging on every update
- Performance: <1ms per update

### Memory Usage
- Entire map loaded into memory
- Typical map: <1MB
- Large map (1000 nodes): ~10MB
- Negligible memory footprint

## Integration Points

### With Map Manager
```
Interactive Editor → saves YAML → Map Manager loads YAML
                                         ↓
                                   Navigation System
```

### With Navigation
```
Interactive Editor → publishes /topological_map_2
                                         ↓
                              Navigation subscribes
                                         ↓
                              Uses updated map
```

### With Validation
```
Interactive Editor → saves YAML → validate_map.py checks
                                         ↓
                                   Reports errors
```

## Extension Points

### Adding New Features

**1. Add/Delete Nodes**:
```python
def add_node(self, name, pose):
    new_node = {
        'meta': {...},
        'node': {
            'name': name,
            'pose': pose,
            'edges': [],
            ...
        }
    }
    self.tmap['nodes'].append(new_node)
    self.create_node_marker(new_node['node'])
    self.server.applyChanges()
```

**2. Edit Properties**:
```python
def update_node_properties(self, node_name, properties):
    for node_data in self.tmap['nodes']:
        if node_data['node']['name'] == node_name:
            node_data['node']['properties'] = properties
            break
```

**3. Edge Visualization**:
```python
def create_edge_markers(self):
    for node_data in self.tmap['nodes']:
        for edge in node_data['node']['edges']:
            # Create line marker from node to edge.node
            self.create_edge_line_marker(node_data, edge)
```

## Debugging

### Enable Debug Logging

```python
# In __init__
self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
```

### Check Marker Server

```bash
# List topics
ros2 topic list | grep topological_map_editor

# Echo marker updates
ros2 topic echo /topological_map_editor/update

# Echo feedback
ros2 topic echo /topological_map_editor/feedback
```

### Verify Map Structure

```python
# In process_feedback()
self.get_logger().debug(f'Map structure: {self.tmap}')
```

## Testing Strategy

### Unit Tests
- Test YAML loading/saving
- Test marker creation
- Test feedback processing
- Test backup/restore

### Integration Tests
- Launch node with test map
- Verify markers appear in RViz2
- Simulate marker movement
- Verify map updates
- Test save functionality

### Manual Tests
- Visual inspection in RViz2
- Interactive manipulation
- Save and reload
- Verify YAML correctness

---

**Architecture Version**: 1.0  
**Last Updated**: 2026-02-10  
**Status**: Stable
