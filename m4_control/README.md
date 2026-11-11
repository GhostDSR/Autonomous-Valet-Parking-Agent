# M4 – Control + Results

```
m4_control/
  pid.py              PID (anti-windup, filtered D)
  path.py             Path/Segment, fillet_polyline (turn A* corners into arcs), gear-change split
  lateral.py          PurePursuit, Stanley (both reverse-capable; Stanley has optional curvature FF)
  follower.py         PathFollower: speed profile + PID + DRIVE/DWELL/DONE state machine
  estimator.py        optional pose filter (dead-reckoning + GPS correction)
  sim.py              bicycle model + actuator limits + sensor noise (offline experiments)
  scenarios.py        synthetic M2-style paths (forward + reverse-in)
  metrics.py          CTE, final pose error, smoothness, success test
  run_experiments.py  E1..E5 -> results/{tables,figures,logs}
  webots_controller.py  Webots adapter (Driver API) + calibrate mode
  plot_log.py         score/plot any run log (sim or Webots) against a path CSV
```

## Run the experiments (no Webots needed)
    pip install numpy pandas matplotlib
    python -m m4_control.run_experiments            # full (20 seeds)   |  --quick for a fast pass

## Interfaces to the other milestones
**From M2** – export CSV `x,y,dir` (dir = +1 forward, -1 reverse; one row per waypoint, rear-axle coordinates, world ground frame):

    path = Path.from_csv("path_m2.csv")
    # or, from a raw A* grid path:  xy = fillet_polyline(astar_xy, radius=6.0)  # >= R_min = L / tan(max_steer)
    #                               path = Path.from_points(xy, direction_flags)

**To/from M3** – M3 owns the high-level states (search → drive → align → park → done); M4 is what runs inside them:

    follower = PathFollower(path, Stanley(L=2.9, use_ff=True), FollowerConfig(max_steer=0.5))
    cmd = follower.step(x, y, yaw, v_signed, dt, estop=m3_emergency_flag)
    # cmd.steer [rad, +left], cmd.accel [m/s², +throttle/-brake in travel direction],
    # cmd.direction (+1/-1 gear), cmd.brake_hold, cmd.done, cmd.remaining [m]
    # M3 -> M4: estop=True brakes at a_brk.   M4 -> M3: cmd.done / cmd.state / cmd.remaining.

## Conventions (the usual source of bugs)
- Pose = REAR-axle centre; yaw = nose direction; steering + = left. Path/slot goals are rear-axle poses.
- Reverse is handled by running the forward law on a "virtual car" (heading yaw+π, steering −δ).
- CTE in results = true rear-axle error for every controller (fair comparison). Forward Stanley tracks with the
  FRONT axle, so its rear axle cuts corners inside fillets – this is visible in the tables, not a bug.

## Webots checklist
1. `compass_yaw_offset` and `steer_sign`: run `mode="calibrate"` once and copy the printed values.
2. `wheelbase`, `max_steer`, `gps_forward_of_rear_axle`: measure from your car PROTO.
3. If throttle/gear don't respond for your vehicle type: `speed_mode="cruise"` (built-in speed control, PID unused).
4. Custom (non-Driver) car: use `ackermann_angles(delta, L, track)` for the two steering motors and send `cmd.v_ref / wheel_radius` to the wheel motors.
5. After a run: `python -m m4_control.plot_log webots_run.csv path_m2.csv` -> same metrics as the offline tables.
