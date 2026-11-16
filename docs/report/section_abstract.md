# Abstract

This project builds an autonomous valet parking agent: a car-like vehicle with Ackermann steering that enters a parking lot, finds a free bay, drives to it and reverses in. It is simulated in Webots with Python controllers. The agent follows the declarative–procedural–reflexive hierarchy. A deliberative layer chooses a bay and plans a route. A procedural layer runs the parking manoeuvre as a state machine. A reflexive layer stops the car whenever an obstacle comes too close. A control layer steers along the route and holds the speed.

For route planning we compare grid A*, A* with smoothing and Hybrid A*. Over six routes, Hybrid A* was the only method whose routes respected the car's 5.13 m minimum turning radius and cleared its footprint, for 8 % more length and about 0.4 s of planning time. For path tracking we compare pure pursuit and the Stanley controller, with and without curvature feed-forward.

Joining the layers exposed dependencies that tests of single modules did not show. We derive where the planned route must end from the geometry of the parking manoeuvre, make the route end there exactly with a cubic curve, and let a reflexive stop trigger deliberative replanning. In our offline simulation the agent parked in 34 of 40 bays (85 %) with both neighbouring bays occupied and without contact in any run. It also recovered when its chosen bay was taken and when a pedestrian blocked the route. Runs of the full agent in Webots on three bays confirmed the offline results.

**Keywords:** autonomous valet parking, intelligent agents, layered architecture, path planning, A*, Hybrid A*, Ackermann steering, pure pursuit, Stanley controller, Webots.
