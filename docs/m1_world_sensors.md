# M1 — Webots World and Sensors

**Module owner:** Devansh Singh Rawat  
**Project:** Autonomous Valet Parking Agent  
**World:** \`worlds/parking_lot.wbt\`  
**Controller:** \`controllers/valet_controller/valet_controller.py\`

## Scope

M1 provides the simulation world, a custom Ackermann-style vehicle model, sensor bring-up, the common pose frame, and LiDAR-based bay classification. It does not issue driving commands: M4 remains the only module that controls vehicle motion.

The world follows the geometry already used by M2 in \`planner/parking_lot.py\`: 40 m × 34 m lot, entrance gap at the left wall between y=6 m and y=11 m, 40 bays (A0–D9), 2.5 m × 5 m bay dimensions, and the centre kerb at y=17 m. The initial car rear-axle pose is (2.0, 8.5, 0.0).

## Coordinate frame and vehicle geometry

- World X/Y are the ground plane; Z is up.
- Heading is radians anticlockwise from world +X.
- The Robot origin and GPS reference are at the centre of the rear axle.
- Vehicle wheelbase: 2.8 m; body: 4.6 m × 1.9 m; rear overhang: 1.0 m.
- Front wheel steering motors are separate so M4 can apply Ackermann inner/outer angles.
- The model exposes four wheel motors and two front steering motors. The M1 controller holds all wheel motors at zero.

These dimensions intentionally match M2's current \`VehicleParams\` placeholders and M3's documented \`VehicleGeometry\`. Measure the final physical/model geometry before claiming calibrated results.

## Devices

| Device name | Webots node | Purpose |
|---|---|---|
| \`gps\` | GPS | World position; mounted at the rear-axle reference |
| \`compass\` | Compass | Heading; converted to CCW from +X |
| \`imu\` | InertialUnit | Available for later yaw/rate fusion |
| \`lidar\` | 360° planar LiDAR | Slot occupancy and directional body clearances |
| \`ds_front\`, \`ds_rear\`, \`ds_left\`, \`ds_right\` | DistanceSensor | Near-field cross-checks |
| \`front_left_steer\`, \`front_right_steer\` | RotationalMotor | Independent front steering |
| \`*_wheel_motor\` | RotationalMotor | Four wheel drives |

## Occupancy logic

\`m1_sensors.py\` converts the planar range image into world-coordinate rays. Each ray is intersected with the oriented footprint of every bay. A return inside the padded parked-car footprint marks the bay occupied. A ray that traverses a bay and returns beyond it provides evidence that the bay is free. A bay with no line-of-sight evidence remains **unknown**; it is not silently treated as free.

The controller emits one JSON record per second prefixed with \`M1_SENSOR\`. It includes rear-axle pose, four estimated body clearances, per-slot states, counts, LiDAR point count, and raw distance-sensor values.

Clearances subtract the vehicle footprint from the LiDAR ray distance. Missing sectors remain \`None\`, consistent with M3's requirement not to substitute arbitrary large clearances for missing data. M3 expects clearances in front/rear/left/right body regions; M4 should pass through the same sensor snapshot rather than manufacture values.

## Run in Webots

1. Install Webots R2023b or later and Python 3.8+.
2. From the repository root, install dependencies: \`pip install -r requirements.txt\`.
3. Open \`worlds/parking_lot.wbt\` in Webots.
4. Confirm the robot controller is set to \`valet_controller\`.
5. Press **Run** and inspect the controller console for \`M1_SENSOR\` lines.
6. Run the pure-Python tests from the repository root:

   \`\`\`bash
   python -m unittest discover -s tests -p "test_m1_sensors.py" -v
   \`\`\`

This is a sensor bring-up controller, not the complete autonomous valet behaviour. M2 route planning, M3 parking, and M4 motion control must be connected by the top-level integration controller.

## Calibration and limitations

- The compass conversion assumes world +Y is north and the Robot's local +X is forward.
- LiDAR sample ordering is assumed to run from -FOV/2 to +FOV/2, as configured by this world.
- A single scan may leave bays unknown due to occlusion. This is intentional; move to a better observation pose or fuse scans before choosing a slot.
- The current clearance estimate uses the M2/M3 rectangular vehicle footprint. Validate it against corners and sensor mounting positions before closed-loop parking.
- The distance sensors are enabled and logged as raw values. Their lookup-table conversion and sensor-specific calibration must be completed before using them as safety-critical clearances.
- No Webots execution result is claimed by the unit tests. Run the world in the installed Webots version and verify sensor names, scan orientation, collision geometry and the actual measured pose.
