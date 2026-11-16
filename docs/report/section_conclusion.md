# Conclusion and Future Work

## Conclusion

We built an agent that parks a car-like vehicle in a simulated parking lot, with a separate layer for deciding, for carrying out the manoeuvre, for reacting and for control. All four modules run together as one agent, offline and in Webots. The main findings are these.

1. **Only a planner that models the car gives drivable routes.** Grid A* was the shortest and about 40 times faster than Hybrid A*, but on average 3.3 m of each route was tighter than the car can turn, and the car's body would have hit an obstacle at four points. Smoothing did not fix this. Hybrid A* had no such stretch and no collision, for 8 % more length.
2. **The tracking controller must suit reversing.** Pure pursuit tracked well in both directions. The plain Stanley controller did not park when reversing, with an RMS tracking error of 0.42 m; adding curvature feed-forward brought that to 0.02 m.
3. **The layers depend on each other.** The planned route has to end where the parking manoeuvre can start: 4 m out from the bay, which follows from the manoeuvre's 6.19 m turning circle. It has to end there to within about 0.3 m, which Hybrid A* alone does not give, so a cubic curve completes it. And the route must leave room for the manoeuvre's set-up move at the last bay of a row.
4. **A reflexive stop works best with deliberative replanning.** When another car took the chosen bay, the agent replanned to a different one. When a pedestrian stepped onto the route, it stopped, replanned twice, waited and then parked. When it could not move at all, it reported that it was boxed in instead of retrying for ever.
5. **Parking success is 85 %, with no contact.** With both neighbouring bays occupied, the agent parked in 34 of the 40 bays. In none of the 40 runs did the car touch anything; the closest it came was 0.07 m. In the six failures the safety layer stopped the car first.
6. **The offline results hold in Webots.** The full agent parked in bays A2, B0 and A5 in Webots, with errors close to the offline ones (about 6 cm and 0.4°). Webots also exposed one fault the offline model could not: the lidar was mounted low enough to see the car's own wheels.

Two lessons go beyond the numbers. First, run the whole chain early. Four problems appeared only when the modules were joined: the reverse swing touching the neighbouring car, the route ending too far from its goal, no room to set up at the end bays, and bays drawn against the outer wall. Each module had passed its own tests. Second, measure every case, not one demonstration. Parking in all 40 bays found faults that a single successful run to one bay had hidden.

## Limitations

- Six bays at the ends of the aisles cannot be parked in yet.
- Webots runs cover three bays, without the pedestrian. The 40-bay result is from the offline simulator.
- The layout of the lot is given to the agent. Only occupancy and obstacles are sensed.
- Pose comes from GPS and a compass, which an indoor car park would not provide.
- The pedestrian is an obstacle that appears and leaves. The agent does not track or predict a moving person.
- Hybrid A* is neither optimal nor complete, and takes up to about 1.3 s for the farthest bays, so the car plans while standing still.

## Future work

- **End-of-aisle bays.** Better tracking through changes of direction, a tighter set-up move, or a Reeds–Shepp connection to the goal [9] should recover most of the six failing bays.
- **More Webots coverage.** Repeat the 40-bay test and the pedestrian scenario in Webots.
- **Moving obstacles.** Track pedestrians and other cars over time, and avoid them locally, for example with the dynamic window approach [17], before falling back to a full replan.
- **Localisation.** Estimate the pose from the lidar and wheel odometry instead of GPS.
- **Smoother routes.** Add the optimisation step of the original Hybrid A* [10], so that the steering angle changes continuously along the route.
- **Other bay types.** Parallel and angled bays, and driving in forwards where there is room.
