## Fluid Navigation - Visual Explanation

### Scenario: Robot navigating from A → B → C → D

```
Start (A) -----> Waypoint (B) -----> Waypoint (C) -----> Goal (D)
```

---

### With `fluid_navigation: true` (Default)

```
     A ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━> D
     🤖                       (B)                  (C)                     STOP
     
     └─ Robot flows smoothly through B and C without stopping
     └─ Ignores exact orientation at B and C
     └─ Only stops at final destination D
```

**Timeline:**
```
t=0s   : Robot starts from A
t=10s  : Robot passes through B (no stop, just passes by)
t=20s  : Robot passes through C (no stop, just passes by)
t=30s  : Robot reaches D and STOPS
```

**Use case:** Efficient navigation through corridors, rows, or long paths

---

### With `fluid_navigation: false`

```
     A ━━━━━> B ━━━━━> C ━━━━━> D
     🤖      STOP     STOP     STOP
     
     └─ Robot stops at each waypoint
     └─ Aligns to exact pose (position + orientation)
     └─ Stops at B, then C, then D
```

**Timeline:**
```
t=0s   : Robot starts from A
t=10s  : Robot reaches B, STOPS and aligns to exact pose
t=15s  : Robot resumes toward C
t=25s  : Robot reaches C, STOPS and aligns to exact pose
t=30s  : Robot resumes toward D
t=40s  : Robot reaches D and STOPS
```

**Use case:** Precise positioning needed (inspection, docking, picking operations)

---

### Mixed Configuration Example

Real-world scenario: Navigate to inspection points in a warehouse

```yaml
edges:
  # Smooth navigation through corridor
  - edge_id: "entrance_hall1"
    fluid_navigation: true   # Flow through
    
  - edge_id: "hall1_hall2"
    fluid_navigation: true   # Flow through
    
  # Stop at inspection point
  - edge_id: "hall2_inspect1"
    fluid_navigation: false  # STOP for inspection
    
  # Continue smoothly
  - edge_id: "inspect1_hall3"
    fluid_navigation: true   # Flow through
    
  # Stop at final destination
  - edge_id: "hall3_storage"
    fluid_navigation: false  # STOP at destination
```

**Visualization:**
```
Entrance ━━━━> Hall1 ━━━━> Hall2 ━━> [INSPECT1] ━━━━> Hall3 ━━> [STORAGE]
   🤖            (pass)     (pass)       STOP          (pass)        STOP
                                          ↓                           ↓
                                     Perform action              Final goal
```

---

### Key Insight

Think of `fluid_navigation` like a traffic signal:

```
fluid_navigation: true   →  🟢 Green light: Keep moving!
fluid_navigation: false  →  🔴 Red light: Stop at this waypoint!
```

The robot automatically stops at the **final destination** regardless of the setting.
