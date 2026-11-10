# M2 – Route Planner: interface and open decisions

Owner: Bhavya Talwar. Code in `planner/`, tests in `tests/`, results in `results/`.

## What it does

Given the parking-lot map, the car's current pose and a free slot, it returns a drivable route from the car to a **pre-park pose** on the aisle next to that slot. The reactive parking manoeuvre (M3) then reverses the car into the bay. The planner does not drive into the bay itself.

## How to call it (M3 / M4 / Webots controller)

```python
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))  # repo root

from planner import build_parking_lot, plan_to_slot

grid_map, slots = build_parking_lot(occupied={"A0", "A1", "B2"})   # ids from free-slot detection
start = (gps_x, gps_y, compass_heading)                           # rear-axle centre, metres / radians
path, stats, goal = plan_to_slot(grid_map, start, slots["C4"])     # default method: Hybrid A*

if not path:
    ...  # no route: pick another slot
for (x, y, heading), direction in zip(path, stats["directions"]):
    ...  # direction: +1 drive forward, -1 reverse
```

- `path` is a list of `(x, y, heading)` poses about 0.25 m apart, in Webots world coordinates.
- `stats["directions"]` matters for path tracking. Hybrid A* may reverse briefly, for example just after the entrance before a tight turn.
- `goal` is the pre-park pose chosen. The path ends within 0.5 m and 10° of it.
- Planning takes about 0.02–1 s on the test lot. Plan once, not every control step.
- To plan to any pose instead of a slot, use `plan_route(grid_map, start, goal_pose)`.
- `method="astar"` or `method="astar_smoothed"` return the same format. They exist for the comparison and are not drivable in general.

## Coordinate frame (proposed)

Webots ENU (the default since R2022a): x and y on the ground, z up. Heading is measured anticlockwise from +x, in radians. Origin is the bottom-left corner of the lot. The car's reference point is the **centre of the rear axle**.

## Decisions needed from M1 (all values in the code are placeholders)

| What | Where in code | Current placeholder |
|---|---|---|
| Lot size, origin, entrance position | `planner/parking_lot.py` | 40 × 34 m, origin bottom-left, entrance gap in the left wall at y = 6–11 m |
| Bay size, positions, ids, rows | `planner/parking_lot.py` | 2.5 × 5 m bays, rows A–D, 10 bays each, ids `A0`…`D9` |
| Kerb between rows B and C | `planner/parking_lot.py` | 0.2 m kerb at y = 17 m |
| Car model dimensions | `planner/vehicle.py` `VehicleParams` | wheelbase 2.8 m, max steer 0.5 rad, 4.6 × 1.9 m, rear overhang 1.0 m |
| Is the Webots car's origin at the rear axle? | `planner/vehicle.py` | assumed yes, to be checked |
| Grid resolution | `build_parking_lot(resolution=...)` | 0.25 m |
| Where the car starts | `ENTRANCE_POSE` | (2.0, 8.5, 0.0) |

## For M3

The pre-park pose is on the aisle centre line, 3 m past the bay, facing along the aisle (see `pre_park_poses` in `planner/parking_lot.py`). The offset and side can be changed to suit the reversing manoeuvre.

## Running

```bash
pip install -r requirements.txt
python -m unittest discover -s tests       # unit tests
python -m planner.compare_planners         # comparison tables + figures -> results/
python planner/astar.py                    # small standalone A* demo
```
