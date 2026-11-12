# World and Sensors (M1)

This section describes the simulated parking lot, the car model and the sensors that the other modules depend on. It also records what went wrong when the world was first run, because the problems changed how we check it.

## The lot

The lot is a Webots world (`worlds/parking_lot.wbt`) built to the same numbers as the planner's map (`planner/parking_lot.py`) [18]. It is 40 m × 34 m, with a 5 m gap in the left wall at y = 6–11 m as the entrance. There are 40 bays, A0–D9, each 2.5 m wide and 5 m deep, in four rows. Rows B and C sit back to back on an island with a 0.24 m kerb, so a car cannot cut across it. Rows A and D stand 0.5 m in from the lot boundary, 0.25 m clear of the wall's inner face. This matters for the sensor: a 4.5 m car centred in a 5 m bay that starts at the wall would have its rear on the wall's inner face, and a lidar return from the wall could not be told from a return from the car. Twenty-two bays hold parked cars, the same set the planner uses as its default.

World and planner must describe the same lot, so `tests/test_m1_world.py` reads the world file and checks every bay line, every row edge and every parked car against `planner/parking_lot.py`.

## The car

The car is a dynamic four-wheel Ackermann vehicle, 4.6 m × 1.9 m, with a 2.8 m wheelbase and a 1.6 m track. Pose is the centre of the rear axle, the same reference M2, M3 and M4 use. All four wheels have a motor and the front pair steers, within ±0.65 rad. At the planner's largest bicycle angle of 0.5 rad the inner front wheel needs 0.57 rad, so the limit leaves room. M1 itself never drives the car. M4 owns the motors; M1 provides a throw-away bring-up controller, `m1_actuator_check`, that checks the signs once.

## Sensors

| Sensor | Use |
|---|---|
| GPS, compass, IMU | Rear-axle position and heading (counter-clockwise from +x) |
| 360° planar lidar, 721 rays, 40 m | Slot occupancy and body clearances. The scan plane is 0.58 m above the floor, so it hits walls and parked cars but not the kerb or the paint lines |
| Four distance sensors (front, rear, left, right) | Near-field cross-check; logged raw, not yet calibrated |
| Steering and wheel position sensors | Steering angle and wheel odometry |

Three conventions had to be fixed before the readings were usable. Webots returns a lidar image clockwise, so it is reversed before use. The compass returns world north in the robot's frame, so heading is `atan2(north_x, north_y)`. The lidar needs no rotation, because Webots devices already use a forward–left–up frame.

## Free-slot detection

For each bay the detector casts every lidar ray against the bay's footprint. A return inside the parked-car footprint, padded by 0.2 m, marks the bay *occupied*. A ray that crosses the bay and lands beyond it shows the bay is visible and empty, so the bay is *free*. A bay that no ray reaches stays *unknown*. It is never assumed free, because a wrong "free" sends the car to a bay that is taken.

Wall-backed bays exposed a flaw. The test accepted a return up to 0.2 m beyond the padded footprint, so a wall just behind an empty bay could mark it occupied. A parked car never reaches past its bay's back line, so the detector now counts only returns that land inside the bay itself.

## What running the world showed

We ran the world in Webots R2023b, headless, and checked it three ways.

1. **Actuators.** `m1_actuator_check` passed 5 of 5. Driving forward moved the car 2.82 m (expected about 3), a left command turned it +24.0° against +25.3° from the bicycle model, the steering sensors matched the command to three decimals (0.327 and 0.277 rad), and reversing moved it −2.79 m.
2. **Slot detection with the real lidar.** From the entrance pose (2.0, 8.5), over 167 one-second reports the controller classified 13 bays occupied, 11 free and 16 unknown. It never called a free bay occupied or an occupied bay free. Row A, directly in front of the car, was resolved completely: A0, A1, A3, A4, A6 and A8 occupied, A2, A5, A7 and A9 free. The unknown bays are the ones no ray reached from that pose, which is the intended behaviour for a single scan.
3. **Without a simulator.** `tests/test_m1_world_scan.py` ray-casts a 721-ray scan against the world file's walls and cars and checks what the detector reports from the aisle in front of each parked car and each free wall-side bay. The tests run in CI, where Webots is not available.

The first run did not pass. Three faults in the world came to light:

- **Missing ground.** The world had no `EXTERNPROTO` declarations, so Webots R2023b skipped the floor and the background when it loaded the file. With no ground, the car fell, and the actuator check passed only 2 of 5 (the "straight keeps heading" and "steering sensors" checks passed because they do not need movement). Declaring the three prototypes fixed it.
- **Cars lying across their bays.** Each of the 22 parked-car boxes was 4.5 m wide in x, in a bay 2.5 m wide. Six pairs of neighbouring cars overlapped by 2 m, and their boxes covered the free bays between them, so even a correct detector would have seen those bays as occupied. Rotating each car a quarter turn fixed it. The ray-cast tests fail on the original file.
- **Rows against the wall.** Moving rows A and D 0.5 m off the wall, as in the planner, removed the wall from the bays' footprints.

None of these shows up in a unit test of the detector alone. Each was found by running the world, or by checking it against the planner's map, which is why the world-file tests exist.

## Limitations

- The Webots runs used software rendering and fast mode, not real time. The actuator figures are from one run on one machine.
- One scan from the entrance leaves 16 of 40 bays unknown. The car must move to a better viewpoint, or combine scans, before it chooses a slot. A "free" reading comes from a ray that crosses the padded footprint, so a ray that only clips a corner of an occupied bay's padding can still call it free. We saw this once in the ray-cast checks (B5, from the far aisle across the island) and did not change it. From the aisle in front of the bay, which is where the car parks from, every bay the tests check was classified correctly.
- Tyre friction, motor torque and suspension are not calibrated. The distance sensors are not converted to metres.
