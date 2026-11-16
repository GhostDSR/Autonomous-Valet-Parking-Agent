# Introduction

## Background

Parking is a demanding part of everyday driving. The space is tight, the car must reverse, and other cars and people are close by. It is also one of the most suitable driving tasks to automate: speeds are low, the area is small, and its layout is known in advance. In automated valet parking the driver leaves the car at the entrance, and the car finds a free bay and parks itself.

The task is a good test of an intelligent agent because no single method solves it. The car has to *decide* which bay to use and how to reach it, which needs a map and a planner. It has to *carry out* a reverse-parking manoeuvre, which is a fixed procedure. And it has to *react* at once when something comes too close, which leaves no time to plan. These three needs are the declarative, procedural and reflexive levels of our syllabus, and they match the layered architectures used in mobile robotics [2], [3].

Planning for a car is also harder than planning for a point robot. A car cannot turn on the spot, so the shortest path on a grid is usually one it cannot drive. Hybrid A* was developed for this kind of semi-structured space, in the DARPA Urban Challenge [10], [12], and it is the planner our agent uses.

## Problem statement

The course brief for project P13 asks for a car-like (Ackermann) vehicle that finds a free slot in a parking lot and parks itself, using a deliberative planner for the route and reactive rules for the close-range manoeuvres.

We made this precise. The lot is 40 m × 34 m with 40 perpendicular bays in four rows. By default 22 bays hold parked cars and 18 are free. The car, 4.6 m long and 1.9 m wide, starts at the entrance. A run succeeds when the car passes the checks M3 applies before it reports that parking is complete, and has hit nothing on the way. That is, when the car:

- is at rest inside a free bay;
- has its body centre within 0.15 m of the bay centre;
- is within 3° of the bay's axis;
- has not touched a wall, the kerb, a parked car or a pedestrian on the way.

The agent must also cope with two changes while it drives: another car taking the bay it chose, and an obstacle appearing on its route.

## Objectives

1. Build the parking lot, the car and its sensors in Webots, and detect free bays from the lidar scan (M1).
2. Plan a route from the entrance to a free bay that the car can drive, and compare grid A*, A* with smoothing and Hybrid A* (M2).
3. Reverse into the bay with a state machine, and stop safely when an obstacle is too close (M3).
4. Follow the route and hold the speed, and compare pure pursuit with the Stanley controller (M4).
5. Join the four modules into one agent, and measure how often it parks, in every bay and when the lot changes.

## Syllabus topics covered

| Topic | Where it appears |
|---|---|
| Declarative–Procedural–Reflexive hierarchy | The planner (declarative), the parking state machine and path tracker (procedural), and the safety stops (reflexive) |
| Intelligent agents | A goal-based agent with a sense–plan–act loop; sensors, actuators and environment as in Russell and Norvig [1] |
| Path planning | A* on an occupancy grid, path smoothing, Hybrid A* |
| Moving | The kinematic bicycle model, Ackermann steering, pure pursuit, Stanley and PID speed control |

## Scope and assumptions

- The work is in simulation: Webots [18], and an offline simulator of our own that runs the same agent code.
- The layout of the lot is known. Which bays are occupied, and any other obstacle, is sensed with the lidar.
- The car's pose comes from GPS and a compass. We do not estimate position from the lidar.
- Bays are perpendicular, and the car reverses in.
- Speeds are low: 1.0 m/s on the route, 0.5 m/s when a route reverses, and 0.45 m/s while parking.
- Parked cars do not move. The pedestrian is an obstacle that appears on the route and later leaves.

## System overview

Each layer is one module, owned by one member of the group.

| Module | Layer | What it does | Runs |
|---|---|---|---|
| M1 – World and Sensors | Sensing | The Webots lot and car; turns the lidar scan into free bays and obstacles | Every step |
| M2 – Route Planner | Deliberative | Chooses a bay; plans a drivable route to a pose beside it; replans when the lot changes | Once, then on change |
| M3 – Parking Manoeuvres | Procedural and reflexive | State machine for the reverse park; brakes when a gap is too small | Every step |
| M4 – Control and Results | Control | Steers along the route and holds the speed | Every step |

```
        lidar scan, GPS, compass
                  |
   M1  sensing    |   free bays, new obstacles
                  v
   M2  DELIBERATIVE   choose a bay -> Hybrid A* route to the pre-park pose
                  |                         ^
                  v                         |  stop on the route -> replan
   M3  PROCEDURAL     DRIVE -> ALIGN -> PARK -> DONE
       REFLEXIVE      brake if a gap is under 0.15 m or under the stopping distance
                  |
                  v
   M4  CONTROL        steering and speed commands -> wheel and steering motors
```

One class, `ValetAgent`, runs this loop. The same class runs in Webots and in the offline simulator. Only the source of the sensor readings differs, so results measured offline apply to the code that runs in Webots.

## Organisation of the report

The Literature Review (Section 3) covers the methods the agent builds on. Novelty (Section 4) presents what is new, with the measurements for each point. The module sections (World and Sensors, Route Planner, Parking Manoeuvres, Control) describe how each layer works. The Results section gives the experiments and comparisons, and the Conclusion lists the limits and future work. Contribution (Section 10) states who did what, and References (Section 13) lists the sources.
