# M1 — Webots World and Sensors

**Module owner:** Devansh Singh Rawat  
**Project:** Autonomous Valet Parking Agent  
**World:** \`worlds/parking_lot.wbt\`  
**Controller:** \`controllers/valet_controller/valet_controller.py\`

## Scope

M1 provides the simulation world, a drivable four-wheel Ackermann vehicle model (steering joints, wheels and motors), the sensors, the common pose frame, and LiDAR-based bay classification. M1 does not issue driving commands: M4 remains the only module that controls vehicle motion. The one exception is the throw-away bring-up check \`controllers/m1_actuator_check/\`, which is never part of a pipeline run.

The world follows the geometry already used by M2 in \`planner/parking_lot.py\`: 40 m × 34 m lot, entrance gap at the left wall between y=6 m and y=11 m, 40 bays (A0–D9), 2.5 m × 5 m bay dimensions, and the centre kerb at y=17 m. The initial car rear-axle pose is (2.0, 8.5, 0.0).

## Coordinate frame and vehicle geometry

- World X/Y are the ground plane; Z is up.
- Heading is radians anticlockwise from world +X.
- The Robot origin and GPS reference are at the centre of the rear axle.
- Vehicle wheelbase: 2.8 m; body: 4.6 m × 1.9 m; rear overhang: 1.0 m.
- The Webots Robot is a dynamic four-wheel vehicle. Body frame is FLU (+x forward, +y left, +z up). Track 1.6 m (wheel centres at y = ±0.8 m), wheel radius 0.3 m, wheel width 0.25 m, wheel mass 20 kg, chassis mass 1200 kg. The Robot origin sits at rear-axle height (0.3 m above the floor), so GPS and Robot translation are the rear-axle pose.
- Drive: all four wheels have a motor; steering is on the front axle only. Positive wheel velocity drives the car forward, positive steering angle turns it left (counter-clockwise).
- Steering joints are limited to ±0.65 rad. At the planner's maximum bicycle angle of 0.5 rad the inner front wheel needs 0.57 rad and the outer 0.44 rad (\`ackermann_angles(delta, L=2.8, T=1.6)\`).
- The chassis is three boxes (rear/mid, narrow front section, bumper) so the steered front wheels have room to turn without touching the body. The cabin is visual only and sits above the lidar plane.

These dimensions intentionally match M2's current \`VehicleParams\` placeholders and M3's documented \`VehicleGeometry\`. Measure the final physical/model geometry before claiming calibrated results.

## Devices

| Device name | Webots node | Purpose |
|---|---|---|
| \`gps\` | GPS | World position; mounted at the rear-axle reference |
| \`compass\` | Compass | Heading; converted to CCW from +X |
| \`imu\` | InertialUnit | Available for later yaw/rate fusion |
| \`lidar\` | 360° planar LiDAR | Slot occupancy and directional body clearances; scan plane 0.58 m above the floor (hits walls and parked cars, not the kerb) |
| \`ds_front\`, \`ds_rear\`, \`ds_left\`, \`ds_right\` | DistanceSensor | Near-field cross-checks |
| \`front_left_steer\`, \`front_right_steer\` | RotationalMotor | Independent front steering, position control, ±0.65 rad, positive = left |
| \`front_left_wheel_motor\`, \`front_right_wheel_motor\`, \`rear_left_wheel_motor\`, \`rear_right_wheel_motor\` | RotationalMotor | Four wheel drives; \`setPosition(inf)\` then \`setVelocity(rad/s)\`, max 60 rad/s, 400 N·m |
| \`front_left_steer_sensor\`, \`front_right_steer_sensor\`, \`front_left_wheel_sensor\`, \`front_right_wheel_sensor\`, \`rear_left_wheel_sensor\`, \`rear_right_wheel_sensor\` | PositionSensor | Steering angle and wheel rotation (odometry) |

## Occupancy logic

\`m1_sensors.py\` converts the planar range image into world-coordinate rays. Each ray is intersected with the oriented footprint of every bay. A return inside the padded parked-car footprint marks the bay occupied. A ray that traverses a bay and returns beyond it provides evidence that the bay is free. A bay with no line-of-sight evidence remains **unknown**; it is not silently treated as free.

The controller emits one JSON record per second prefixed with \`M1_SENSOR\`. It includes rear-axle pose, four estimated body clearances, per-slot states, counts, LiDAR point count, and raw distance-sensor values.

Each clearance is the smallest straight-line distance from a LiDAR hit point in that sector (±25° around front/left/rear/right) to the vehicle footprint rectangle (3.6 m ahead of and 1.0 m behind the rear axle, 0.95 m each side). Missing sectors remain \`None\`, consistent with M3's requirement not to substitute arbitrary large clearances for missing data. M3 expects clearances in front/rear/left/right body regions; M4 should pass through the same sensor snapshot rather than manufacture values.

## Sensor conventions (checked against the Webots R2022a+ documentation)

- ENU world: x east, y north, z up. The compass returns world north (+Y) in the robot frame, so heading = atan2(north_x, north_y). (The first version had the sign of north_x flipped, which is invisible at the spawn heading of 0.)
- Webots devices are FLU, so the lidar needs no mounting rotation for a horizontal scan. (The first version rotated it 90° about x, which would have made the scan vertical.)
- Webots returns a lidar range image clockwise, from the sensor's left to its right. The controller reverses it with \`webots_ranges_to_ccw\` before calling the M1 functions, which assume counter-clockwise order from -FOV/2 to +FOV/2.
- The rear distance sensor is rotated by π so it faces backwards.

## Run in Webots

1. Install Webots R2023b or later and Python 3.8+.
2. From the repository root, install dependencies: \`pip install -r requirements.txt\`.
3. Open \`worlds/parking_lot.wbt\` in Webots.
4. Confirm the robot controller is set to \`valet_controller\`.
5. Press **Run** and inspect the controller console for \`M1_SENSOR\` lines.
6. Check the actuators once: set the car's \`controller\` field to \`m1_actuator_check\`, run, and read the \`M1_ACTUATOR_CHECK PASS/FAIL\` lines (forward is +x, positive steer turns left, reverse is -x, steering sensors follow the command). Set the controller back to \`valet_controller\` afterwards.
7. Run the pure-Python tests from the repository root:

   \`\`\`bash
   python -m unittest discover -s tests -v
   \`\`\`

This is a sensor bring-up controller, not the complete autonomous valet behaviour. M2 route planning, M3 parking, and M4 motion control must be connected by the top-level integration controller.

## Calibration and limitations

- The compass conversion assumes world +Y is north and the Robot's local +X is forward.
- LiDAR sample ordering is assumed to run from -FOV/2 to +FOV/2, as configured by this world.
- A single scan may leave bays unknown due to occlusion. This is intentional; move to a better observation pose or fuse scans before choosing a slot.
- The current clearance estimate uses the M2/M3 rectangular vehicle footprint. Validate it against corners and sensor mounting positions before closed-loop parking.
- Tyre friction, motor torque and suspension are not calibrated. The wheels are rigid with default Webots contact properties; if the car jitters at the 32 ms basic time step, reduce \`basicTimeStep\` to 16.
- The distance sensors are enabled and logged as raw values. Their lookup-table conversion and sensor-specific calibration must be completed before using them as safety-critical clearances.
- The static world checks in \`tests/test_m1_world.py\` do not run Webots. The world file, lidar orientation and ordering, compass sign and the actuator signs above have not yet been run in Webots; verify them with the steps above. Run the world in the installed Webots version and verify sensor names, scan orientation, collision geometry and the actual measured pose.
