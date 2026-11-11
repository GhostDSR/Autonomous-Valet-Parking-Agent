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

- The car must be a Webots `Car` / `AckermannVehicle`, so that `vehicle.Driver` works. The controller uses M4's `DriverCar` adapter for steering, throttle, brake and gear.
- The devices must be named `gps`, `compass` and `lidar` (360°, one layer). The GPS must sit at the rear-axle centre.
- Run M4's calibrate mode once to confirm the steering sign.

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

| Owner | Issue | Suggested fix |
|---|---|---|
| **M1 (Devansh)** | `worlds/parking_lot.wbt` still has rows A and D against the wall. | Move rows A and D (bay lines and parked cars) 0.5 m away from the wall: row A centre y 2.5 → 3.0, row D centre y 31.5 → 31.0. This matches `planner/parking_lot.py`. |
| **M1** | The car has no wheels, steering or motors. | Make it a `Car` / `AckermannVehicle` (Driver API). Wheelbase 2.8 m, max steer 0.5 rad, as in `planner/vehicle.py`. |
| **M1** | `body_clearances_from_scan` subtracts `front·\|cos\| + half_width·\|sin\|`. This is a safe lower bound, but very loose for side rays: with a car 0.6 m beside ours it reports 0 m, so M3 stops the car in every bay that has a neighbour. | Use the exact distance from each lidar point to the car rectangle, as in `body_clearances` in `agent/valet_agent.py`. The agent uses that version for now. |
| **M1** | `test_body_clearance_subtracts_vehicle_footprint` expects 6.4 m. With obstacles 10 m away all round, the nearest are off the front corners, so the true gap is `10 − hypot(3.6, 0.95) = 6.28` m. | Fix the expected value. The function's 6.277 is right for this case. |
| **M1** | `classify_slots_from_scan` pads each bay by 0.2 m. For bays backed by a wall, rays that hit the wall land inside the padded box, so free A/D bays read as occupied. | Use less padding along the bay depth, or ignore returns beyond the bay's back line. |
| **M1** | `compass_yaw` returns `atan2(−n_x, n_y)`. For a car with heading yaw, the compass reads north as `(sin yaw, cos yaw)`, so the heading is `atan2(n_x, n_y)`. M4's adapter already uses that. | Flip the sign. Confirm with M4's calibrate mode. |
| **M1** | Check that Webots' lidar range image runs from −FOV/2 to +FOV/2, as `m1_sensors.py` assumes. Webots may order it the other way. | Check once in Webots with an object on the car's left. |

## Results

Tables and figures are in `results/agent/` (`agent_scenarios.md`, `agent_sweep.md`, `agent_*.png`).
