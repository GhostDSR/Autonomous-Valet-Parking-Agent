# 3. Literature Review

This section reviews the work our agent builds on, in the order the agent uses it: how an intelligent agent is organised, how a route is planned on a map, how a route is made drivable for a car, and how the car follows it.

## 3.1 Intelligent agents and layered control

Russell and Norvig define an intelligent agent as something that perceives its environment through sensors and acts on it through actuators. They range agents from simple reflex agents, which map the current percept straight to an action, to goal-based agents, which search for a sequence of actions that reaches a goal [1]. A valet-parking car needs both. It must decide where to go, which is goal-based. It must also react at once when something gets too close, which is reflex.

Brooks argued against one central planner. In his subsumption architecture, simple reactive layers run in parallel, and higher layers can override lower ones [2]. Reactive layers respond quickly and cope well with surprises, but on their own they cannot plan a long route through a parking lot.

Gat's three-layer architecture combines the two ideas [3]. It has a *deliberator* that plans slowly and far ahead, a *sequencer* that runs step-by-step procedures, and a *controller* that reacts within each control cycle. This matches the declarative–procedural–reflexive hierarchy in our syllabus, and it is the structure of our agent:

- the route planner (M2) is the deliberative layer;
- the parking state machine (M3) is the procedural layer, with its safety checks as the reflexive layer;
- the path tracker (M4) is the low-level controller.

## 3.2 Path planning on a map

Hart, Nilsson and Raphael introduced A* [4]. It finds a lowest-cost path in a graph by expanding nodes in order of `f = g + h`. Here `g` is the cost so far and `h` is an estimate of the remaining cost. A* is guaranteed to find an optimal path if `h` never overestimates the remaining cost (an *admissible* heuristic). On an occupancy grid, A* is the standard method for global route planning. LaValle's textbook covers it together with the configuration-space idea: inflate the obstacles by the robot's size so that the robot can be planned for as a point [5]. Our grid A* uses both.

Sampling-based planners are the other main family. Probabilistic roadmaps [6] and RRT* [7] connect random samples of the configuration space and suit high-dimensional problems. For a flat parking lot with a known map, grid and lattice search are simpler and give repeatable results, so we did not use sampling.

## 3.3 Planning for a car: kinematic constraints

A car cannot turn on the spot. With wheelbase `L` and maximum steering angle `δ_max`, its tightest turn has radius `R_min = L / tan(δ_max)`. Dubins found the shortest paths for a car that only drives forward with a bounded turning radius [8]. Reeds and Shepp extended this to a car that can also reverse [9]. Their paths are made of circular arcs and straight lines, and they are widely used as an exact "last step" to a goal pose.

Grid A* ignores the turning radius, so its paths have sharp corners that a car cannot drive. Two fixes are common:

1. **Smooth the path afterwards.** Dolgov *et al.* refine their routes with a numerical optimisation that trades off smoothness, distance from obstacles and curvature [10]; a simpler version, which our project uses for grid A*, pulls each point towards its neighbours. Smoothing rounds corners off, but because curvature is only a penalty in the cost, it does not guarantee the turning radius is respected.
2. **Plan with the car's motion built in.** Pivtoraiko *et al.* search a *state lattice* of pre-computed drivable motions [11]. Dolgov *et al.* proposed **Hybrid A\*** for the Stanford entry in the DARPA Urban Challenge [10], [12]. It keeps a continuous `(x, y, heading)` state, expands it by driving short arcs of the car model, and stores at most one state per cell of a coarse 3D grid. Its heuristic is the larger of two estimates: a 2D distance that goes around obstacles but ignores the turning radius, and a distance that respects the turning radius but ignores obstacles. It finishes with an analytic Reeds–Shepp step to the goal. Hybrid A* was designed for parking lots and other semi-structured spaces, which is exactly our setting.

Paden *et al.* survey these planning methods, the controllers in Section 3.4 and the vehicle models they rely on, for self-driving vehicles [13]. A lesson we draw from this, and saw in our own integration, is that the planner and the controller must use the same vehicle model for a route to be drivable.

## 3.4 Following the route

Pure pursuit steers the car towards a point a fixed "look-ahead" distance along the path, using the arc that passes through that point [14]. It is simple and stable, but cuts corners when the look-ahead is long. The Stanley controller, from the DARPA Grand Challenge winner, steers by the heading error plus a term in the cross-track error measured at the front axle [15], [16]. It tracks more tightly at low speed. A PID loop is the usual way to hold a target speed. Our M4 module compares pure pursuit and Stanley, and the M3 parking manoeuvre uses pure-pursuit geometry when reversing into the bay.

## 3.5 Reacting to obstacles

Local reactive methods keep the car safe between plans. The dynamic window approach picks velocities that the robot can reach within one control step and that keep it clear of obstacles [17]. Our reflexive layer is simpler. It compares each measured gap around the car body with the stopping distance at the current speed, and brakes if the gap is too small. The deliberative layer then plans a new route around the obstacle. This is the layered split described by Gat [3].

## 3.6 Simulation

Webots is a robot simulator with physics, sensor models (lidar, GPS, compass) and a vehicle library for Ackermann cars [18]. Testing in simulation lets us run the same scenario many times and compare methods fairly, which would be costly and unsafe with a real car.

## 3.7 Gap addressed by this project

The papers above each study one layer: a planner, a controller, or an architecture. Course projects usually show a planner on its own. What is less often shown is how the layers depend on each other in a parking task:

- where the global route must end so that the local manoeuvre can succeed;
- how exactly it must end there;
- what the deliberative layer should do after the reflexive layer stops the car.

Our project builds all three layers and measures these links (Section 4).
