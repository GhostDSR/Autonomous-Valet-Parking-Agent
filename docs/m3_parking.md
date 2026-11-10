# M3 — Parking Manoeuvres

**Module owner:** Harshil Mittal  
**Project:** Autonomous Valet Parking Agent  
**Status:** Standalone Python behaviour implemented; 19 test methods passed locally. Webots integration and experiments remain to be completed with M1 and M4.

## Purpose and module boundary

M3 controls the final perpendicular reverse-parking manoeuvre. It accepts a free bay from M1 and the pre-parking pose chosen by M2, waits for M4 to finish following the route, then supplies low-speed parking requests to M4.

M1 supplies sensor processing and vehicle/world geometry. M2 supplies the global route. M4 follows that route, applies the final steering and speed/brake commands, and records experiment results. M3 provides the state machine, local manoeuvre, clearance checks and parking-completion decision.

## Files

| File | Responsibility |
|---|---|
| `behaviours/__init__.py` | Python package marker. |
| `behaviours/parking_types.py` | Shared states, observations, geometry and commands. |
| `behaviours/parking_fsm.py` | State transitions, route handoff and latched stops. |
| `behaviours/parking_safety.py` | Input validation, clearance checks, limits and body-in-bay checks. |
| `behaviours/parking_manoeuvre.py` | Forward setup, reverse turn, straightening and bounded corrections. |
| `tests/test_parking.py` | State-machine tests and kinematic motion tests. |

## State machine

```text
SEARCH -> DRIVE -> ALIGN -> PARK -> DONE
                   |        |
Active states can also end in EMERGENCY_STOP or FAILED.
```

| State | Behaviour and exit condition |
|---|---|
| `SEARCH` | Hold stopped until the main controller has a selected slot and a successful M2 route, then calls `select_slot`. |
| `DRIVE` | Ask M4 to follow the selected M2 route. When route following finishes, require a stop and check arrival near the intended pre-parking pose. |
| `ALIGN` | Move forward to a turning pose calculated from the car geometry. Stop and check position and heading before reversing. |
| `PARK` | Reverse along a quarter-circle followed by a straight segment. Use pose feedback for steering and slow near segment ends. If needed, stop, move forward and try a bounded alignment correction. |
| `DONE` | Hold stopped after verifying final position, heading, speed and the complete vehicle footprint. |
| `EMERGENCY_STOP` | Brake and hold after unsafe clearance or invalid/stale required sensor information. |
| `FAILED` | Brake and hold after a timeout, unsupported geometry, incorrect route endpoint, tracking failure or exhausted corrections. |

`DONE`, `FAILED` and `EMERGENCY_STOP` remain latched. To retry, first hold the vehicle stopped, call `fsm.reset()`, and select a new route from the actual current pose. Clearing an obstacle does not automatically resume the old manoeuvre.

## Coordinate frame and data contract

Use one agreed ground-plane frame throughout M1–M4:

- Positions and distances: metres; time: simulation seconds.
- Heading: radians, increasing anticlockwise from world +x.
- Car `Pose`: centre of the rear axle.
- `ParkingSlot.x` and `.y`: centre of the bay; `.yaw`: desired heading of the parked car.
- Speed: m/s, positive forward and negative in reverse.
- Internal steering: radians, positive left.

M1 must transform GPS mounting offsets and compass readings into this frame. M2's proposed frame must match the actual Webots world. A GPS reading at the body centre is not automatically a rear-axle pose.

The rear-axle parking target is computed from the bay centre:

```text
centre_offset = vehicle_length / 2 - rear_overhang
rear_target_x = slot_x - centre_offset * cos(slot_yaw)
rear_target_y = slot_y - centre_offset * sin(slot_yaw)
```

### Required observation each simulation step

| `ParkingObservation` field | Meaning |
|---|---|
| `timestamp_s` | Time the required sensor snapshot was measured. Use the oldest required measurement if sensor timestamps differ. |
| `pose` | Rear-axle `Pose(x, y, yaw)`, or `None` if unavailable. |
| `speed_mps` | Measured signed speed, or `None` if unavailable. |
| `front_clearance_m` | Minimum obstacle clearance from the front body region, including corners. |
| `rear_clearance_m` | Minimum obstacle clearance from the rear body region, including corners. |
| `left_clearance_m` | Minimum clearance from the left side and relevant corners. |
| `right_clearance_m` | Minimum clearance from the right side and relevant corners. |
| `route_complete` | M4's Boolean flag that the selected M2 route has finished. |

All four clearance values must be valid during an active task. Supply finite, nonnegative clearances; `None`, NaN and infinity are rejected. When no obstacle is detected, M1 must use a validated finite clear sensing range or report the reading unavailable. Do not replace unknown coverage with an arbitrary large distance. These clearances are derived body clearances, not unadjusted sensor ranges.

### Command handling in M4

| `ParkingCommand.mode` | Required M4 action |
|---|---|
| `FOLLOW_ROUTE` | Follow the already selected M2 route and its per-pose travel directions. M3's speed and steering fields are `None` in this mode. |
| `PARKING` | Use M3's signed `target_speed_mps` and `steering_rad`, respecting the car's actuator limits. M4's speed controller applies the speed request. |
| `STOP` | Override movement requests, brake to a stop, and keep holding. A `None` steering value does not grant permission to continue moving. |

M4 is the single writer of vehicle commands. It must stop before changing direction, including at reversing points in M2's route. Keep calling the supervisor each simulation step, even after completion or a stop, so STOP continues to be applied. Log `command.state` and `command.reason`.

The Webots Driver API uses positive steering for **right**, while M3 uses positive steering for **left**. At the Driver adapter, use `driver_steering_rad = -command.steering_rad` for parking requests. Driver speed readings and cruising-speed requests use km/h: divide readings by 3.6 for M3 and multiply M3 speed requests by 3.6 only when passing them to a km/h API. For torque/PID control, handle reverse gear after stopping. Select one speed-control strategy for the vehicle. See the [official Webots Driver documentation](https://github.com/cyberbotics/webots/blob/master/docs/automobile/driver-library.md).

## Construction and small API example

Run this example from the repository root. It uses example geometry and **synthetic clearances in open space** to demonstrate the API; it is not a Webots controller. Replace the dimensions with the selected M1 vehicle before integration.

```python
import math
from dataclasses import replace

from behaviours.parking_fsm import ParkingFSM
from behaviours.parking_manoeuvre import ManoeuvreConfig, ParkingManoeuvre
from behaviours.parking_safety import SafetyConfig
from behaviours.parking_types import (
    ParkingObservation, ParkingSlot, Pose, VehicleGeometry,
)

vehicle = VehicleGeometry(
    wheelbase_m=2.8,
    length_m=4.6,
    width_m=1.9,
    rear_overhang_m=1.0,
    max_steer_rad=0.5,
)
safety_config = SafetyConfig()
rules = ParkingManoeuvre(
    vehicle,
    config=ManoeuvreConfig(),
    safety_config=safety_config,
)
fsm = ParkingFSM(vehicle, manoeuvre=rules, safety_config=safety_config)

slot = ParkingSlot("A2", 10.0, 2.5, math.pi / 2, 2.5, 5.0)
staging = Pose(13.0, 8.5, 0.0)
fsm.select_slot(slot, staging)

observation = ParkingObservation(
    timestamp_s=0.0,
    pose=staging,
    speed_mps=0.0,
    front_clearance_m=10.0,
    rear_clearance_m=10.0,
    left_clearance_m=10.0,
    right_clearance_m=10.0,
    route_complete=True,
)
command = fsm.update(observation, now_s=0.0)
print(command.state.value, command.mode.value)

observation = replace(observation, timestamp_s=0.032)
command = fsm.update(observation, now_s=0.032)
print(command.state.value, command.mode.value)
```

Expected output:

```text
align stop
align parking
```

The first call confirms arrival and holds a stop. The next call begins the forward setup phase. Passing the same `SafetyConfig` to the supervisor and manoeuvre keeps their limits consistent. Construct these objects once per controller instance, rather than every simulation step.

## Handoff from M2

M2's inspected `plan_to_slot` interface returns `(path, stats, goal)`. A successful path ends near a pre-parking pose, not inside the bay. M4 must receive `path` and `stats["directions"]`, since the route may contain reversing segments.

M2's `Slot` type and M3's `ParkingSlot` are different. The following is an integration fragment, after M2 has returned a nonempty `path` and a non-None `goal`, and the supervisor is fresh/reset:

```python
m3_slot = ParkingSlot(
    slot_id=planner_slot.slot_id,
    x=planner_slot.x,
    y=planner_slot.y,
    yaw=planner_slot.yaw,
    width_m=bay_width_m,
    depth_m=bay_depth_m,
)
fsm.select_slot(m3_slot, Pose(*goal))
```

Here `planner_slot` is the selected M2 slot; bay dimensions come from the agreed M1 world geometry. M4 should have the corresponding route ready before the main loop processes `FOLLOW_ROUTE`. If planning fails, hold STOP and choose another slot. M3 does not call M2 or load its occupancy grid itself.

M2's `VehicleParams` and M3's `VehicleGeometry` also have different field names. Build both from the same measured M1 dimensions; passing one class directly in place of the other is unsupported. The interface used here is documented in [M2's handoff notes at the inspected commit](https://github.com/bhavyatalwar04/Autonomous-Valet-Parking-Agent/blob/62abdf26e766d979ada6b766fbb782f805f0ef52/docs/m2_route_planner.md).

### Why ALIGN moves farther forward

M3 calculates a reverse quarter-circle using the wheelbase and steering limit, with steering headroom for feedback. It then checks that space remains for a straight segment into the bay.

For the example car, the base turning radius is `2.8 / tan(0.85 * 0.5)`, approximately **6.19 m**. Starting 3 m past the bay therefore typically requires about **3.19 m** more forward travel before reversing. This is an example based on placeholder dimensions, not a fixed distance for every vehicle.

The forward setup and possible 2.5 m correction must have room in the actual world. Near an aisle end, another approach direction or another bay may be needed. Successful global planning to the pre-parking pose alone does not prove the whole local parking manoeuvre is collision-free.

## Checks and configurable starting values

| Setting | Default | Meaning |
|---|---|---|
| `ManoeuvreConfig.speed_mps` | 0.35 m/s | Maximum requested manoeuvre speed. |
| `ManoeuvreConfig.lookahead_m` | 0.65 m | Local steering lookahead. |
| `ManoeuvreConfig.tracking_limit_m` | 0.6 m | Stop with failure after excessive path deviation. |
| `ManoeuvreConfig.correction_distance_m` | 2.5 m | Forward correction target distance from the final rear-axle target, along the parked heading. |
| `ManoeuvreConfig.max_corrections` | 2 | Maximum correction attempts; zero disables corrections. |
| `ManoeuvreConfig.phase_timeout_s` | 70 s | Maximum duration of each local phase. |
| `SafetyConfig.clearance_margin_m` | 0.15 m | Minimum current body clearance. |
| `SafetyConfig.max_sensor_age_s` | 0.2 s | Maximum permitted sensor snapshot age. |
| `SafetyConfig.braking_deceleration_mps2` | 1.0 m/s² | Assumed achievable braking deceleration. |
| `SafetyConfig.max_parking_speed_mps` | 0.5 m/s | Supervisor limit on parking speed requests. |
| `SafetyConfig.task_timeout_s` | 180 s | Maximum total active task time, including DRIVE. |

The front/rear stopping-distance check uses:

```text
required_clearance = margin + speed * reaction_time + speed² / (2 * braking_deceleration)
```

For local movement requests, `speed` is the larger magnitude of observed and requested speed. The relevant front/rear regions are checked for current and requested travel directions. Left/right regions also retain the minimum body-clearance check. The braking assumption and sensor coverage must be verified with M1 and M4.

Default parking success requires all of the following:

- Absolute measured speed at most 0.03 m/s.
- Heading error at most 3 degrees.
- Vehicle-body centre within 0.15 m of the bay centre.
- All four body corners inside the bay with a 0.05 m inset.

These values are configurable simulation starting points. Confirm that the centred target preserves clearance from real kerbs, walls and parked vehicles; a painted bay rectangle alone does not describe those obstacles.

## Tests and validation status

From the repository root, run:

```bash
python -m unittest discover -s tests -p "test_parking.py" -v
```

M3 tests use only Python's standard library. The 19 test methods cover route handoff, waiting for a stop, incorrect arrival, missing/stale inputs, latched emergency stops, requested speed/steering limits, direction changes, timeouts, parking completion and local motion.

Motion cases include both bay sides, both aisle directions, a rotated frame, small initial heading errors, slower steering response, obstacle interruptions, recoverable lateral disturbances and disabled corrections. The bicycle model uses a 0.032 s step, limited acceleration/braking and limited steering rate. Most motion checks use constant synthetic clearances representing open space.

**Completed:** all 19 methods passed locally.  
**Pending:** execution on the uploaded GitHub branch, actual Webots vehicle/world integration, sensor calibration, collision checks against neighbouring cars/kerbs, and final project experiments. Passing this suite is not a measured Webots parking success rate.

The manoeuvre supports perpendicular bays. Parallel parking, arbitrary tight-lot manoeuvres and a guarantee for the complete future swept vehicle footprint are outside the current implementation. The local tracker uses [pure-pursuit geometry described by Coulter](https://publications.ri.cmu.edu/implementation-of-the-pure-pursuit-path-tracking-algorithm); M4 remains responsible for global route tracking and actuator control.

## Integration experiments with M1 and M4

1. Agree on the world frame, rear-axle pose, vehicle dimensions and bay dimensions.
2. Test stopping and signed steering/speed conventions at low speed.
3. Park into one empty bay from a fixed staging pose.
4. Add neighbouring parked cars and check front swing, sides and rear coverage.
5. Repeat with pose offsets, both aisle approaches and obstacles introduced during parking.
6. Check that emergency stops remain held until an explicit reset and replan.
7. Record time, state, manoeuvre phase, pose, speed, steering, body-centre error, heading error, minimum clearance, correction count and final outcome. M4 uses these logs for plots and comparison tables.

Project results such as Webots parking time, collisions and success rate should be reported only after these experiments have been performed.
