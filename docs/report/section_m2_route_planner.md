# Route Planner (M2)

This section describes the deliberative layer of the agent: how it represents the lot, chooses a free bay and plans a route the car can actually drive. The measured comparison of the planners is in Section 4 and `results/planner_comparison.md`.

## Role in the agent

The planner answers two questions: *which free bay should the car go to*, and *how does it get there*. It is slow compared with the control loop (up to about a second per plan), so it runs once at the start and again only when something changes. It does not drive the car into the bay. Its route ends at a **pre-park pose** on the aisle, and the parking manoeuvre (M3) reverses in from there.

| Input | From |
|---|---|
| Which bays are occupied | M1 free-slot detection |
| Current pose (x, y, heading) of the rear axle | M1 GPS and compass |
| Extra obstacles seen by the lidar (for example a pedestrian) | M1 lidar scan |

| Output | Used by |
|---|---|
| Chosen bay and its pre-park pose | M3 |
| Route: poses about 0.25 m apart, each marked forward or reverse | M4 path tracking |

## The map

The lot is an occupancy grid with 0.25 m cells (`planner/grid_map.py`, `planner/parking_lot.py`): 160 × 136 cells for the 40 m × 34 m lot. A cell is either free or occupied. Walls, the kerb between rows B and C, and the parked cars are drawn in as rectangles. The grid uses the same frame as Webots (x and y on the ground, origin at the lot's bottom-left corner), so a pose from the GPS can be used without conversion. An occupancy grid suits this problem because the lot is small, flat and mostly static, and both A* and Hybrid A* work directly on it [5].

## The car model

The planner uses the kinematic bicycle model, with the pose taken at the centre of the rear axle. Driving a distance `d` with steering angle `δ` changes the heading by `d · tan(δ) / L`, where `L = 2.8 m` is the wheelbase. With the steering limit `δ_max = 0.5 rad`, the tightest circle the car can drive has radius

`R_min = L / tan(δ_max) = 5.13 m`,

so no drivable route can bend more sharply than `1 / R_min = 0.195 m⁻¹`.

For collision checks, the 4.6 m × 1.9 m body is covered by three circles of radius 1.32 m along its centre line (`planner/vehicle.py`). The obstacles are grown by that radius once, when the map is built. Checking a pose is then three cell look-ups, which matters because Hybrid A* checks hundreds of thousands of poses per plan. The circles overhang the corners of the body slightly, so the check errs on the safe side.

## Grid A*

A* searches the grid cell by cell, always expanding the cell with the lowest `f = g + h`, where `g` is the distance travelled so far and `h` is an estimate of the distance left [4] (`planner/astar.py`). We use 8-connected moves (cost 1 straight, √2 diagonal) and the octile distance as the heuristic. Octile distance is the exact cost on an empty 8-connected grid, so it never overestimates, and A* returns the shortest grid path. Diagonal moves that would cut the corner of an obstacle are not allowed.

A* treats the car as a disc centred on the rear axle and has no notion of heading. Its paths turn 45° or 90° on the spot, which a car cannot do. We keep it for two reasons: as the baseline in the comparison, and as a fast way to estimate route lengths when choosing a bay.

## A* with smoothing

The second method smooths the A* path by gradient descent (`planner/smoothing.py`). Each interior point is pulled towards its original position and towards the midpoint of its neighbours, and a move that would enter an obstacle is undone. This rounds the corners, but it gives no guarantee about the turning radius: Section 4 shows the smoothed paths still contain stretches tighter than the car can turn.

## Hybrid A*

Hybrid A* [10] searches over car poses `(x, y, heading)` instead of cells (`planner/hybrid_astar.py`). Our implementation works as follows.

- **Motion primitives.** From each pose the search drives the bicycle model 1 m with one of five steering angles (full left, half left, straight, half right, full right), forward or in reverse. That gives ten successors per pose. Every route is a chain of these arcs, so it respects the turning radius by construction.
- **Binning.** Poses are continuous, so they are grouped into bins of 0.5 m × 0.5 m × 5°. Only one pose is expanded per bin. This keeps the search finite while the poses themselves stay exact.
- **Cost.** The cost of a step is its length, multiplied by 2 in reverse, plus small penalties for steering, for changing the steering angle, and a penalty of 5 for each switch between forward and reverse. The planner therefore prefers calm forward driving and reverses only where it must.
- **Heuristic.** Before the search, one Dijkstra pass from the goal gives every cell its shortest obstacle-free distance to the goal. This guides the search along the aisles and round the island instead of into walls.
- **Collision checks.** Each arc is checked every 0.25 m with the three-circle model.
- **Stopping.** The search stops when a pose is within 0.5 m and 10° of the goal, or reports failure after 100 000 expansions.

Compared with the original paper we made three simplifications. We do not use the Reeds–Shepp heuristic [9], only the obstacle-aware one. We do not use an analytic Reeds–Shepp connection to the goal; a simpler curve does that job (below). And we do not post-optimise the route.

## The pre-park pose

M3 reverses into a bay on a quarter circle of radius 6.19 m. The pre-park pose is where the planner hands over, so it must leave room for that arc. It is placed 4 m out from the bay's entrance line and 3 m past the bay along the aisle, facing along the aisle (`pre_park_poses`). Each bay has two such poses, one for each direction of approach. The planner also checks that there is room for M3's forward set-up move, about 6.5 m beyond the bay; if that would run into the end wall, it uses the other direction. Section 4.2 gives the geometry behind the 4 m.

## Exact final approach

The 0.5 m goal tolerance of Hybrid A* is too loose for the hand-over: the room for error across the aisle is about 0.3 m. With `final_approach_m = 5`, the search instead ends at a point 5 m before the pre-park pose, on the same line, and a cubic curve joins the two (`planner/final_approach.py`). In the goal's frame, with `u` along the aisle and `v` across it, the curve is

`v(t) = v₀ + m₀ t + a t² + b t³`, with `a = −(3v₀ + 2m₀D) / D²` and `b = (2v₀ + m₀D) / D³`,

where `v₀` and `m₀` are the sideways offset and slope at the end of the searched route and `D` is the distance left. It ends exactly on the goal with zero offset and zero slope. Every point of the curve is checked against the curvature limit and for collisions. If either check fails, the search continues to another candidate.

## Choosing a bay and replanning

Choosing a bay needs a route cost for each free bay. Running Hybrid A* for all of them would take many seconds, so the choice has two stages (`planner/replanning.py`). Grid A* first ranks every free bay by route length, at a few milliseconds each. Hybrid A* then plans to the best one, and moves down the list only if no drivable route exists.

While the car drives, the route is checked once a second against the latest scan. Three events lead to a new plan:

| Event | Response |
|---|---|
| The chosen bay has been taken | Choose a new bay from the car's current pose |
| An obstacle blocks the route ahead | New route to the same bay, or a new bay if it cannot be reached |
| The safety layer stopped the car | Replan from where it stands |

If nothing can be planned, for example while a pedestrian stands in the aisle, the agent waits and tries again after 5 s. In the integrated agent, routes are planned with a 0.4 m safety margin around the body circles to absorb tracking error. If the car has stopped somewhere too tight for that, one more attempt is made with no margin.

## Interface

The other modules call two functions. `plan_to_slot(grid_map, start_pose, slot)` returns the route, statistics (including the forward/reverse flag for every pose) and the pre-park pose used. `choose_slot` and `replan` wrap it for the integrated agent. All three planning methods return the same format, so the comparison in Section 4 runs them through the same code path.

## Testing

The planner runs in plain Python without Webots. Sixty unit tests cover the grid, A*, Hybrid A*, the final approach, bay choice and replanning (`tests/`). Among other things they check that every Hybrid A* route stays within the curvature limit, clears the body's footprint and ends at the goal. They run on every pull request in GitHub Actions. `python -m planner.compare_planners` regenerates the comparison tables and figures.

## Limits

- Planning takes up to about 1.3 s for the farthest bays, so the car plans while standing still.
- The route has constant-curvature arcs, so the steering angle jumps between them. M4's tracking absorbs this at parking-lot speeds.
- The map assumes the lot's layout is known. Only occupancy and extra obstacles come from the sensors.
- Six bays at the aisle ends cannot yet be parked in (Section 4.6). Reaching them needs multi-point turns in tight space.
