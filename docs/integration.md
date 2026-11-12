# Integration: the whole valet agent

Owner: Bhavya Talwar. Code in `agent/` and `controllers/valet_agent/`.

This puts the four modules together into one agent and tests the result end to end, both offline and (once the car has motors) in Webots.

## How the modules fit

```
           lidar scan, GPS, compass
                     |
   M1  sensing       |  classify_slots_from_scan, lidar_points_world  ->  free slots, new obstacles
                     v
   M2  DELIBERATIVE  choose_slot -> Hybrid A* route to the pre-park pose      (about 1 s, rarely)
                     re-check the route every 1 s: slot taken? route blocked?
                     |                               ^
                     v                               | emergency stop on the route -> replan
   M3  PROCEDURAL    state machine  DRIVE -> ALIGN -> PARK -> DONE
       REFLEXIVE     safety: stop if any gap < 0.15 m or < stopping distance   (every step)
                     |
                     v
   M4  CONTROL       Stanley / pure pursuit + speed PID -> steering, throttle, brake
```

`agent/valet_agent.py` holds this loop: `actuation = agent.step(now, pose, speed, lidar_ranges)`. The same class runs offline and in Webots. Only the source of the sensor data differs.

| Phase | What happens |
|---|---|
| `SEARCH` | Stopped. Read the slots from the lidar, pick a slot (grid A* ranks them, Hybrid A* plans the best one), plan the route. |
| `DRIVE` | M4 follows the route. Every second the route is re-checked against the latest scan. |
| `STOPPING` | The slot was taken, the route is blocked, or M3 made an emergency stop. The car brakes to a standstill and replans from where it is: same slot if it is still free, otherwise the next best one. If nothing is reachable (for example a pedestrian is in the way), it waits and tries again every second. |
| `ALIGN`, `PARK` | M3's manoeuvre into the bay. No replanning here. M3's safety layer stops the car if needed. |
| `DONE`, `FAILED`, `EMERGENCY_STOP` | Final. |

## Running it

Offline, with no Webots needed. It uses the real M1–M4 code with a simulated lidar (`agent/sim_world.py`) and M4's bicycle model:

```bash
python -m agent.run_offline            # 3 scenarios -> results/agent/
python -m agent.run_offline --sweep    # + park in every one of the 40 bays (about 10 min)
python -m unittest discover -s tests   # all tests
```

In Webots, set the car's `controller` field to `valet_agent`. The world needs:

- It uses the devices M1 added in #11: `gps`, `compass`, `lidar`, the four `*_wheel_motor`s and the two `front_*_steer` motors.
- The wheel motors are velocity controlled, so the controller sends M4's speed reference and steering angle, converted to wheel speeds and Ackermann angles with M1's helpers.
- Run `m1_actuator_check` once first, to confirm the motor signs in your Webots version.

## Agent settings that matter (`AgentConfig`)

| Setting | Value | Why |
|---|---|---|
| `cruise_speed` | 1.0 m/s | M3 stops the car if the gap ahead is less than its stopping distance. At 1.5 m/s that is 1.6 m, which tight corners do not have; at 1.0 m/s it is 0.85 m. |
| `reverse_speed` | 0.5 m/s | The car strays further from the route when reversing through tight turns (up to 0.77 m at 1.0 m/s). |
| `planning_margin_m` | 0.4 m | Routes keep this extra distance from obstacles to absorb tracking error. If the car stops somewhere tight and nothing can be planned with it, the agent retries with no extra margin. |
| `retry_period_s` | 5 s | A failed search is slow, so after one the agent waits before trying again (for example, until a pedestrian moves). |

If every new route is stopped by M3 again before the car moves 10 cm (three times in a row), the agent reports `FAILED: boxed in`. In that state M3 allows no motion at all.

## Changes made in M2 to make parking work

These came out of running the whole chain. They are all in `planner/`.

1. **The pre-park pose is 4 m out from the bay entrance**, not on the aisle centre line (`pre_park_poses`). M3 reverses in on a 6.19 m quarter circle. From the centre line, the inner side of that swing reached the parked car in the next bay (3 cm gap, then contact). From 4 m out the gap is about 0.2 m.
2. **The route ends exactly on the pre-park pose** (`HybridAStarConfig.final_approach_m`, `planner/final_approach.py`). Hybrid A* only gets within 0.5 m. The room for error across the aisle is about 0.3 m, so the agent plans to a point 5 m before the goal and joins the two with a checked cubic curve.
3. **Approach direction is checked for M3's set-up room** (`plan_to_slot(..., setup_distance_m=...)`). M3 drives about 6.5 m past the bay before reversing. For the last bay in a row, that runs into the end wall, so the planner now approaches from the other side.
4. **Rows A and D moved 0.5 m off the outer wall** (`ROWS` in `planner/parking_lot.py`). The bays started at the wall itself, so a centred 4.6 m car would end with its rear inside the 0.25 m wall.

## For teammates: issues found while integrating

Fixed by M1 in #11: motors and steering on the car, the clearance formula and its test, the compass sign, and the lidar scan order. The integrated controller now uses `compass_yaw`, `webots_ranges_to_ccw`, `ackermann_angles` and `wheel_speeds` from M1.

Still open:

| Owner | Issue | Suggested fix |
|---|---|---|
| **M1 (Devansh)** | ~~`worlds/parking_lot.wbt` still has rows A and D against the outer wall.~~ **Fixed:** rows A and D are 0.5 m off the wall (row A centre y 3.0, row D centre y 31.0), matching `planner/parking_lot.py`. | Guarded by `tests/test_m1_world.py`. |
| **M1** | ~~`classify_slots_from_scan` marks free wall-side bays occupied.~~ **Fixed:** returns beyond a bay's back line are ignored. | Guarded by `tests/test_m1_sensors.py` and `tests/test_m1_world_scan.py`. |
| **M1** | `body_clearances_from_scan` only covers ±25° around each axis, so obstacles off the corners are missed. A side with no return gives `None`, which M3 treats as a fault. | Assign each point to its nearest side, and report "no return" as clear to max range (see `body_clearances` in `agent/valet_agent.py`). |
| **M3 / M4** | Six bays at the ends of the aisles still fail in the sweep. | See Results. |

## Results

Tables and figures are in `results/agent/` (`agent_scenarios.md`, `agent_sweep.md`, `agent_*.png`).
