# 4. Novelty

Each layer of our agent uses a known method: A* and Hybrid A* for planning, a state machine for the parking manoeuvre, and Stanley / pure pursuit for tracking. What is new is how the layers were joined and measured. Building the whole chain showed several dependencies between the global planner and the local manoeuvre that single-layer studies do not show. We solved them with small, explainable additions and measured each one.

## 4.1 A drivability-focused comparison of three planners

We compare grid A*, grid A* with gradient smoothing, and Hybrid A* on the same lot, start and goals. Path length and planning time are the usual measures, but they do not show whether a car can drive a route. We added measures that do:

- maximum curvature, computed separately for each forward and reverse stretch, so a direction change is not counted as a sharp turn;
- the length of route that is tighter than the car can turn;
- how often the car's real rectangular footprint hits an obstacle along the route;
- the heading error at the goal.

Results over six routes (mean, except maximum curvature, which is the worst route; `results/planner_comparison.md`):

| Method | Length (m) | Time (ms) | Max curvature (1/m) | Too-tight length (m) | Footprint hits | Goal heading error (°) |
|---|---|---|---|---|---|---|
| A* | 28.7 | 9.9 | 2.53 | 3.27 | 4.0 | 52.5 |
| A* + smoothing | 28.0 | 21.6 | 1.06 | 3.46 | 1.8 | 44.7 |
| Hybrid A* | 31.0 | 372.9 | 0.20 (= car limit) | 0.00 | 0.0 | 3.4 |

Grid A* gives the shortest routes, about 40 times faster, but none of them can be driven as planned. Smoothing makes the corners rounder but does not fix the problem: the length of route tighter than the car can turn even goes up slightly. Only Hybrid A* gives routes that respect the turning radius and clear the car's footprint, for 8 % more length.

## 4.2 A pre-park pose derived from the manoeuvre's geometry

The planner does not drive into the bay. It stops at a "pre-park" pose on the aisle, and the reactive layer reverses in from there. Placing that pose on the aisle centre line seemed natural, but in the integrated run the reverse swing touched the parked car in the next bay. The geometry explains why. The manoeuvre reverses on a quarter circle of radius `R = L / tan(0.85 δ_max) = 6.19 m`, and the neighbouring car lies inside that turn. It must therefore stay within the innermost circle the body sweeps, radius `R − w/2 = 5.24 m`. From the centre line, 3.5 m out from the bay, the neighbour's far corner is exactly 5.24 m from the circle's centre, so the gap is zero. From 4.0 m out it is 0.2 m, and the simulation measured 0.19 m. The pre-park pose is now worked out from the manoeuvre rather than chosen by eye.

## 4.3 An exact final approach without Reeds–Shepp curves

Hybrid A* only promises to reach the goal within 0.5 m, but the geometry above leaves about 0.3 m of room across the aisle. Instead of the usual Reeds–Shepp step [9], [10], we search to a point 5 m before the goal and join it to the exact goal with a cubic curve, checked against the turning limit and for collisions. This works because every pre-park pose lies on a straight aisle, facing along it. It is a few lines of algebra, easy to explain, and every route now ends exactly on the goal (used for every bay in the sweep below).

## 4.4 Two-stage slot choice

Choosing a slot needs a route cost for every free slot. Hybrid A* takes about a second per slot, so we rank all free slots with fast grid A* (a few ms each) and then run Hybrid A* only on the best one, moving to the next only if it fails. The slot choice takes about one Hybrid A* run instead of one per free slot.

## 4.5 Reflexive stops that trigger deliberative replanning

The three layers interact in a defined way:

- the reflexive layer (M3 safety) stops the car whenever a gap around the body is below its stopping distance;
- once the car is stopped, the deliberative layer (M2) rebuilds the map from the lidar and plans again from where the car is, to the same slot if it is still free, otherwise to the next best;
- if every new route is stopped again before the car can move, the agent reports that it is boxed in rather than looping.

Offline scenarios (`results/agent/agent_scenarios.md`):

| Scenario | What happens | Result |
|---|---|---|
| Normal | Picks the best slot, drives, reverses in | Parked in A2 after 53 s; closest gap 0.20 m |
| Slot taken | Another car takes our slot 3 s into the drive | Replans to B0; parked after 55 s; closest gap 0.21 m |
| Pedestrian | Steps onto the route 6 m ahead, leaves after 10 s | Two replans; parked in A2 after 78 s; closest gap 0.23 m |

## 4.6 Measured parking success in every bay

Rather than one demonstration, we tried to park in each of the 40 bays with both neighbours occupied (the tightest case). That run found the problems behind Sections 4.2 and 4.3, plus two more:

- bays drawn against the outer wall left no room behind a parked car;
- the last bay in a row has no room to set up the manoeuvre when approached towards the end wall.

Final success rate per row (`results/agent/agent_sweep.md`):

| Row | Parked | Rate |
|---|---|---|
| A | 9 / 10 | 90 % |
| B | 9 / 10 | 90 % |
| C | 7 / 10 | 70 % |
| D | 9 / 10 | 90 % |
| **All** | **34 / 40** | **85 %** |

In none of the 40 runs did the car touch anything: the closest it came was 0.07 m.

The sweep was rerun after M3's parking speed and lookahead were tuned in Webots (0.35 → 0.45 m/s, 0.65 → 0.55 m; Section 4.7). The success rate and the failing bays did not change, the mean time to park fell from 77.0 s to 68.6 s (11 %), and the largest final heading error fell from 0.26° to 0.11°. Every parked car ended within 5.3 cm of the bay centre.

The six bays that still fail (A8, B8, C0, C8, C9, D1) are at or next to the ends of the aisles. There, either reaching the bay needs a multi-point turn in a narrow space, or the reverse swing passes the end wall within the 0.15 m safety limit. In every case the safety layer stops the car before contact. Better path tracking through direction changes (M4) and a tighter set-up for end bays (M3) are the next steps.

## 4.7 Offline results confirmed in Webots

The offline runs use our own simulated lidar and a bicycle model of the car, so the same agent was also run in Webots, where M1 senses the bays from the real lidar scan and M4 drives the four wheel and two steering motors (`results/m3_webots_tuning/`, `docs/m3_webots_tuning.md`). The first Webots runs exposed one problem the offline model could not: with the lidar mounted 0.28 m above the rear axle, it saw the car's own wheels, so the map showed obstacles around the car and no route could be planned. Raising it to 0.35 m, above the tyres and below the cabin, fixed this.

M3 then compared its old and tuned parking settings in Webots on three bays, starting from the lot entrance each time (time from the start of the parking manoeuvre to parked):

| Bay | Parking time, old → tuned (s) | Centre error, old → tuned (cm) | Heading error, old → tuned (°) | Closest lidar gap, tuned (m) |
|---|---|---|---|---|
| A2 | 45.2 → 36.7 | 6.3 → 5.9 | 0.76 → 0.40 | 0.18 |
| B0 | 46.0 → 37.2 | 6.3 → 6.0 | 0.74 → 0.36 | 0.21 |
| A5 | 45.2 → 36.7 | 6.3 → 5.9 | 0.78 → 0.40 | 0.20 |

All five validation runs with the tuned settings parked, about 19 % faster, with the heading error halved. The Webots errors are close to the offline ones (about 5 cm and well under 1°), which supports using the offline sweep for the 40-bay success rate. Webots has so far been run on three bays and without moving obstacles; the remaining bays and the pedestrian scenario are still to be repeated there.
