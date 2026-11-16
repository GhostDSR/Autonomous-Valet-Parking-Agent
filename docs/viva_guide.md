# Viva guide: the whole system in questions and answers

Short answers to the questions an examiner is likely to ask, for all four modules. Numbers are from `results/` and the report as of 16 Nov 2026. Each answer says where to look in the code.

## 1. The project in one minute

A car enters a 40-bay parking lot, finds a free bay, drives to it and reverses in, without hitting anything, and copes when the bay is taken or someone steps into its way.

It is built in four layers, each owned by one member:

| Layer | Module | Does what | How often |
|---|---|---|---|
| Sensing | M1 (Devansh) | Webots world, car, lidar; says which bays are free | every step |
| Deliberative | M2 (Bhavya) | Picks a bay and plans a drivable route | once, then on change |
| Procedural + reflexive | M3 (Harshil) | Parking state machine and safety stops | every step |
| Control | M4 (Shahvez) | Steers along the route, holds the speed | every step |

`agent/valet_agent.py` joins them in one loop: `actuation = agent.step(time, pose, speed, lidar)`.

## 2. Syllabus links

**Q. What kind of agent is this?**
A goal-based agent with a layered (hybrid) architecture. It has a goal (parked in a free bay), a model of the world (the grid map), and it plans. It is not purely reactive, because a reactive agent could not find a route round the island; and it is not purely deliberative, because planning takes up to a second and a pedestrian will not wait.

**Q. Where is the Declarative–Procedural–Reflexive hierarchy?**

| Level | In our system | Example |
|---|---|---|
| Declarative (knows *what*) | M2: map, free bays, the chosen bay and route | "Go to A2 along this route" |
| Procedural (knows *how*) | M3 state machine, M4 tracking | "Drive forward past the bay, reverse on an arc, straighten" |
| Reflexive (reacts at once) | M3 safety layer | "Gap ahead is less than the stopping distance: brake" |

The levels talk to each other: a reflexive stop makes the declarative level replan.

**Q. PEAS?**
Performance: parked inside the bay, straight, no contact, in little time. Environment: the parking lot, parked cars, a pedestrian. Actuators: four wheel motors, two steering joints. Sensors: GPS, compass, 360° lidar.

**Q. What is the environment like?**
Partially observable (the lidar cannot see behind cars), dynamic (bays get taken, pedestrians move), continuous, single agent. That is why it needs replanning and a reflex layer.

## 3. M2: Route planner

**Q. How is the lot represented?**
An occupancy grid with 0.25 m cells, 160 × 136 for the 40 × 34 m lot. A cell is free or occupied. Same coordinate frame as Webots. (`planner/grid_map.py`, `planner/parking_lot.py`)

**Q. How does A* work?**
It keeps a priority queue of cells ordered by `f = g + h`: `g` is the cost so far, `h` is an estimate of the cost left. It always expands the cell with the lowest `f`. If `h` never overestimates (it is *admissible*), the first time the goal is taken from the queue the path is the shortest.

**Q. Which heuristic, and why?**
Octile distance: `max(dx, dy) + (√2 − 1) · min(dx, dy)`. It is the exact cost on an empty grid with diagonal moves, so it is admissible and as informed as possible. Manhattan would overestimate with diagonals; Euclidean would underestimate more and expand more cells.

**Q. A* vs Dijkstra?**
Dijkstra is A* with `h = 0`. It expands in all directions. A* expands towards the goal, so it visits fewer cells and gives the same path.

**Q. Why is A* not enough for a car?**
A* moves between cell centres and turns 45° or 90° on the spot. A car has a minimum turning radius: `R = L / tan(δ_max) = 2.8 / tan(0.5) = 5.13 m`. A* also has no heading, so it cannot arrive facing the right way, and it treats the car as a disc. In our test its routes had on average 3.3 m that were too tight to drive and 4 footprint collisions.

**Q. What does smoothing do, and why does it not fix it?**
It pulls each point towards the midpoint of its neighbours (rounds corners) and back towards its original place (stops drift). Corners get rounder, but nothing enforces the turning radius. The too-tight length did not go down (3.27 → 3.46 m).

**Q. How does Hybrid A* work?**
Same `f = g + h` search, but the state is a car pose `(x, y, heading)`.
1. From each pose, drive the bicycle model 1 m with 5 steering angles, forward or reverse: 10 successors. Every route is made of drivable arcs.
2. Poses are continuous, so they are binned (0.5 m, 0.5 m, 5°), and one pose is expanded per bin. Continuous states with discrete bookkeeping: that is the "hybrid".
3. Cost: length; reverse costs double; small penalties for steering; 5 for each forward/reverse switch.
4. Heuristic: the shortest distance to the goal on the 2D grid round obstacles, computed once by Dijkstra from the goal.
5. Collision check every 0.25 m with three circles covering the body.

**Q. Is Hybrid A* optimal?**
No. Binning throws away poses, and the primitives are a small fixed set. It gives a good drivable route, not the shortest one. It is also not complete in the strict sense: it can fail (we cap it at 100 000 expansions) where a route exists.

**Q. What did you leave out from the original paper (Dolgov et al.)?**
The Reeds–Shepp heuristic, the Reeds–Shepp analytic connection to the goal, and the final smoothing step. We replaced the analytic connection with a cubic curve, which is enough because our goals are always on a straight aisle.

**Q. Results of the comparison?** (six routes, `results/planner_comparison.md`)

| Method | Length | Time | Too-tight length | Footprint hits | Heading error at goal |
|---|---|---|---|---|---|
| A* | 28.7 m | 9.9 ms | 3.27 m | 4.0 | 52.5° |
| A* + smoothing | 28.0 m | 21.6 ms | 3.46 m | 1.8 | 44.7° |
| Hybrid A* | 31.0 m | 372.9 ms | 0 | 0 | 3.4° |

Hybrid A* is about 8% longer and about 40 times slower, and it is the only one a car can drive.

**Q. What is the pre-park pose and why 4 m?**
It is where the route ends and M3 takes over: on the aisle, 4 m out from the bay's entrance line and 3 m past the bay. M3 reverses on a quarter circle of radius `2.8 / tan(0.85 × 0.5) = 6.19 m`. The inside of the body sweeps a circle of radius `6.19 − 0.95 = 5.24 m`, and the neighbouring parked car is inside that turn. From the aisle centre line (3.5 m out) the neighbour's corner is exactly on that circle: zero gap, and the first integrated run touched it. From 4 m the gap is about 0.2 m (0.19 m measured).

**Q. What is the final approach?**
Hybrid A* only gets within 0.5 m of the goal; the hand-over needs about 0.3 m or better. So it plans to a point 5 m before the goal, and a cubic curve `v(t) = v₀ + m₀t + at² + bt³` joins that to the exact goal with zero offset and zero slope. The curve is checked for curvature and collisions. (`planner/final_approach.py`)

**Q. How is the bay chosen?**
Two stages. Grid A* ranks all free bays by route length (milliseconds each). Hybrid A* plans to the best; if that fails, the next. One expensive plan instead of one per free bay. (`planner/replanning.py`)

**Q. When does it replan?**
The route is checked every second against the latest scan. It replans if the bay was taken (new bay), the route is blocked (new route, same bay if possible), or the safety layer stopped the car. If nothing can be planned it waits 5 s and tries again. If it is stopped three times without moving 10 cm, it reports "boxed in" and fails cleanly instead of looping.

**Q. Why not RRT or a lattice planner?**
RRT gives jerky, different routes each run and needs smoothing; the lot is small and structured, where a grid search is reliable and repeatable. A state lattice would also work but needs precomputed primitives; Hybrid A* is simpler to explain and implement.

## 4. M1: World and sensors

**Q. What is in the world?**
A 40 × 34 m lot, entrance in the left wall, 4 rows of 10 bays (2.5 × 5 m), a kerb between the middle rows, 22 parked cars. The car is 4.6 × 1.9 m, wheelbase 2.8 m, four wheel motors, front wheels steer.

**Q. How is a bay classified?**
From the lidar scan: a return inside the bay's footprint means *occupied*; a ray that passes through the bay and lands beyond it means *free*; a bay no ray reaches is *unknown* and is never assumed free.

**Q. What went wrong in Webots that the offline tests missed?**
The lidar was mounted 0.28 m above the rear axle and saw the car's own wheels, so the map had obstacles around the car and no route could be planned. It was raised to 0.35 m. Also fixed earlier: the compass sign, the lidar scan order (Webots returns it clockwise), and rows A and D drawn against the wall.

## 5. M3: Parking manoeuvre and safety

**Q. What are the states?**
`SEARCH → DRIVE → ALIGN → PARK → DONE`, with `EMERGENCY_STOP` and `FAILED` reachable from the active states. In ALIGN and PARK the car drives forward past the bay to set up, reverses on an arc into the bay, then straightens.

**Q. What is the reflexive part?**
Every step, the gap on each side of the body is compared with the distance needed to stop at the current speed. If the gap is smaller, or any gap is 0.15 m or less, the car brakes, whatever the planner wanted.

**Q. What was tuned in Webots?**
Parking speed 0.35 → 0.45 m/s and lookahead 0.65 → 0.55 m. On bays A2, B0 and A5 parking took about 19% less time (45 s → 37 s) and the final heading error halved (0.76° → 0.40°).

## 6. M4: Control

**Q. Pure pursuit in one sentence?**
Pick a point on the route a lookahead distance ahead and steer along the circle that passes through it: `δ = atan(2 L sin α / l_d)`.

**Q. Stanley in one sentence?**
Steer by the heading error plus a term that grows with the cross-track error: `δ = θ_e + atan(k · e / v)`, measured at the front axle.

**Q. What did the comparison show?** (`results/m4/tables.md`, L-shaped route)
Pure pursuit tracked well forward and in reverse (RMS error 0.10 m and 0.02 m). Plain Stanley failed in reverse (0.42 m, did not park). Stanley with curvature feed-forward fixed the reverse (0.02 m).

**Q. And the speed?**
A PID controller on the speed error, with feed-forward from the planned acceleration. The agent cruises at 1.0 m/s and reverses at 0.5 m/s, slow enough that the stopping distance fits the gaps in the lot.

## 7. Integration and results

**Q. How did you test without Webots?**
`agent/sim_world.py` simulates the lot and a ray-cast lidar, and M4's bicycle model moves the car. The agent code is the same one that runs in Webots; only the source of the sensor data differs.

**Q. Scenario results?** (`results/agent/agent_scenarios.md`)

| Scenario | Outcome | Time | Replans | Closest gap |
|---|---|---|---|---|
| Normal | Parked in A2 | 53 s | 0 | 0.20 m |
| Bay taken during the drive | Parked in B0 | 55 s | 1 | 0.21 m |
| Pedestrian on the route | Parked in A2 | 78 s | 2 | 0.23 m |

**Q. How reliable is it?**
We tried every one of the 40 bays with both neighbours occupied: 34 parked (85%), no contact in any run, closest gap 0.07 m. Rows A, B and D: 9/10. Row C: 7/10.

**Q. Why do six fail?**
A8, B8, C0, C8, C9 and D1 are at or next to the ends of the aisles. Reaching them needs multi-point turns in tight space, or the reverse swing passes the end wall inside the 0.15 m safety limit. The safety layer stops the car each time, so they fail safely.

**Q. Was it run in Webots?**
Yes: the full agent parked in A2, B0 and A5 in all five validation runs, with centre error about 6 cm and heading error about 0.4°, close to the offline numbers. The pedestrian scenario and the other bays have not been repeated in Webots yet.

## 8. Honest limits (say these before the examiner does)

- Hybrid A* is neither optimal nor complete, and takes up to about 1.3 s.
- The lot's layout is given, not mapped; only occupancy and obstacles are sensed.
- Position comes from GPS and compass, which a real car park would not have indoors.
- 85% of bays, not 100%; Webots validation covers three bays.
- The pedestrian is a static obstacle that appears and disappears, not a tracked moving object.

## 9. What is new (Section 4)

1. A planner comparison measured on drivability, not only length and time.
2. A pre-park pose derived from the parking manoeuvre's geometry.
3. An exact final approach with a cubic curve instead of Reeds–Shepp.
4. Two-stage bay choice.
5. Reflex stops that trigger replanning, with a clean "boxed in" failure.
6. Parking success measured in all 40 bays.
7. Offline results confirmed in Webots.
