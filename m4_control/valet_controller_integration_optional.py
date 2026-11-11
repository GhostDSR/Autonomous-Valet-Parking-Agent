"""Valet controller: the whole pipeline in Webots.

    M1 sensors  ->  free-slot search     (controllers/valet_controller/m1_sensors.py)
    M2 planner  ->  route to pre-park    (planner/)
    M3 state machine ParkingFSM          (behaviours/)
    M4 control  ->  moves the car        (m4_control/)

The world's car has no wheels or motors, so the car is moved KINEMATICALLY: M4's steering and
acceleration go through the same Ackermann bicycle model used in the offline experiments
(with steering-rate / acceleration limits) and the resulting pose is written to the car node.
This needs `supervisor TRUE` on the car and no `physics` node (see README/chat).

Each run writes results/m4/webots_run_<time>.csv and webots_summary_<time>.json.
"""
import csv
import json
import math
import os
import sys
import time

from controller import Supervisor

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from planner import build_parking_lot, plan_to_slot
from behaviours.parking_fsm import ParkingFSM
from behaviours.parking_manoeuvre import ParkingManoeuvre
from behaviours.parking_types import (ParkingObservation, ParkingSlot, ParkingState, Pose,
                                      VehicleGeometry)
from controllers.valet_controller.m1_sensors import (body_clearances_from_scan,
                                                     classify_slots_from_scan, compass_yaw)
from m4_control.follower import FollowerConfig
from m4_control.lateral import PurePursuit, Stanley
from m4_control.m4_controller import M4Controller
from m4_control.planner_bridge import path_from_planner
from m4_control.sim import Bicycle, VehicleParams

# ------------------------------------------------------------------ settings
CFG = dict(
    lateral="stanley_ff",        # "pure_pursuit" | "stanley" | "stanley_ff"
    route_speed=1.5,             # m/s. M3 needs front clearance > stopping distance (1.6 m at 1.5 m/s)
    pose_source="state",         # "state": use the simulated car state (exact)
                                 # "sensors": GPS + compass_yaw() (needs M1's compass sign verified)
    # vehicle (must match planner/vehicle.py and the world's car body 4.6 x 1.9 m)
    wheelbase=2.8, length=4.6, width=1.9, rear_overhang=1.0, max_steer=0.5,
    # bay size from M2's doc (bays 2.5 x 5 m)
    bay_width=2.5, bay_depth=5.0,
    search_period_s=1.0,
    idle_after_end_s=3.0,
    log_dir=os.path.join(REPO_ROOT, "results", "m4"),
)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class KinematicCar:
    """Bicycle-model state that is written into the Webots car node every step."""

    def __init__(self, node, vp):
        self.t_field = node.getField("translation")
        self.r_field = node.getField("rotation")
        x, y, z = self.t_field.getSFVec3f()
        R = node.getOrientation()                     # 3x3 row-major, world <- body
        self.z = z
        self.car = Bicycle(x, y, math.atan2(R[3], R[0]), vp)

    def step(self, steer, accel, direction, dt):
        c = self.car
        c.step(steer, accel, direction, dt)
        self.t_field.setSFVec3f([c.x, c.y, self.z])
        self.r_field.setSFRotation([0.0, 0.0, 1.0, c.yaw])


def make_lateral(L):
    return {"pure_pursuit": lambda: PurePursuit(L=L),
            "stanley": lambda: Stanley(L=L),
            "stanley_ff": lambda: Stanley(L=L, use_ff=True)}[CFG["lateral"]]()


def main():
    robot = Supervisor()
    ts = int(robot.getBasicTimeStep())
    dt = ts / 1000.0

    gps, compass, lidar = (robot.getDevice(n) for n in ("gps", "compass", "lidar"))
    for d in (gps, compass, lidar):
        d.enable(ts)
    max_range = float(lidar.getMaxRange())
    fov = 2.0 * math.pi

    veh = VehicleGeometry(CFG["wheelbase"], CFG["length"], CFG["width"],
                          CFG["rear_overhang"], CFG["max_steer"])
    vp = VehicleParams(L=veh.wheelbase_m, max_steer=veh.max_steer_rad)
    plant = KinematicCar(robot.getSelf(), vp)
    fsm = ParkingFSM(veh, ParkingManoeuvre(veh))
    fcfg = FollowerConfig(max_steer=veh.max_steer_rad, v_fwd=CFG["route_speed"])

    os.makedirs(CFG["log_dir"], exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    log_path = os.path.join(CFG["log_dir"], f"webots_run_{stamp}.csv")
    fh = open(log_path, "w", newline="")
    wr = csv.writer(fh)
    wr.writerow(["t", "state", "mode", "phase", "x", "y", "yaw", "v", "steer", "accel", "v_ref",
                 "route_cte", "front", "rear", "left", "right", "min_clear", "centre_err_m",
                 "heading_err_deg", "corrections", "slot", "reason"])

    m4, m3_slot = None, None
    next_search_t = 0.0
    compass_checked = False
    end_t = None
    min_clear_seen = float("inf")
    driven = 0.0
    t_first_move = None

    def current_pose():
        if CFG["pose_source"] == "sensors":
            g = gps.getValues()
            return Pose(g[0], g[1], compass_yaw(compass.getValues()))
        return Pose(plant.car.x, plant.car.y, plant.car.yaw)

    print(f"[valet] logging to {log_path}", flush=True)
    while robot.step(ts) != -1:
        t = robot.getTime()
        pose = current_pose()
        v = plant.car.v

        # one-off consistency checks of M1's GPS / compass against the true state
        if not compass_checked and abs(plant.car.yaw) > 0.3:
            compass_checked = True
            err = wrap(compass_yaw(compass.getValues()) - plant.car.yaw)
            flipped = abs(wrap(-compass_yaw(compass.getValues()) - plant.car.yaw)) < 0.05
            print(f"[M1 CHECK] compass_yaw error = {math.degrees(err):+.1f} deg" +
                  ("  -> SIGN IS FLIPPED in m1_sensors.compass_yaw (remove the minus on north_x)"
                   if flipped and abs(err) > 0.05 else "  -> OK" if abs(err) <= 0.05 else ""), flush=True)

        # ------------------------------------------------------------ sensors (M1)
        ranges = lidar.getRangeImage()
        clear = body_clearances_from_scan(ranges, (pose.x, pose.y, pose.yaw), field_of_view=fov)
        # M3 rejects None. "No return in this sector" = clear up to the lidar's max range.
        clear = {k: (max_range if val is None else val) for k, val in clear.items()}
        min_clear = min(clear.values())
        min_clear_seen = min(min_clear_seen, min_clear)

        # ------------------------------------------------------------ SEARCH + PLAN (M1 + M2)
        if m4 is None and t >= next_search_t:
            next_search_t = t + CFG["search_period_s"]
            status = classify_slots_from_scan(ranges, (pose.x, pose.y, pose.yaw),
                                              field_of_view=fov, max_range=max_range)
            free = [sid for sid, s in status.items() if s == "free"]
            blocked = {sid for sid, s in status.items() if s != "free"}
            grid_map, planner_slots = build_parking_lot(occupied=blocked)
            free.sort(key=lambda sid: math.hypot(planner_slots[sid].x - pose.x,
                                                 planner_slots[sid].y - pose.y))
            print(f"[valet] t={t:.1f}s  free={len(free)}  occupied="
                  f"{sum(s == 'occupied' for s in status.values())}  unknown="
                  f"{sum(s == 'unknown' for s in status.values())}", flush=True)
            for sid in free:
                ps = planner_slots[sid]
                path, stats, goal = plan_to_slot(grid_map, (pose.x, pose.y, pose.yaw), ps)
                if not path or goal is None:
                    continue
                cand = ParkingSlot(sid, ps.x, ps.y, ps.yaw, CFG["bay_width"], CFG["bay_depth"])
                try:
                    fsm.select_slot(cand, Pose(*goal))
                except ValueError as err:
                    print(f"[valet] slot {sid} rejected by M3: {err}", flush=True)
                    continue
                m3_slot = cand
                m4 = M4Controller(path_from_planner(path, stats.get("directions")),
                                  make_lateral(veh.wheelbase_m), veh.max_steer_rad, fcfg)
                print(f"[valet] chosen slot {sid}; route {len(path)} poses; pre-park goal "
                      f"({goal[0]:.2f}, {goal[1]:.2f}, {math.degrees(goal[2]):.0f} deg)", flush=True)
                break

        # ------------------------------------------------------------ M3 supervisor
        obs = ParkingObservation(
            timestamp_s=t, pose=pose, speed_mps=v,
            front_clearance_m=clear["front"], rear_clearance_m=clear["rear"],
            left_clearance_m=clear["left"], right_clearance_m=clear["right"],
            route_complete=bool(m4.route_complete) if m4 else False)
        cmd = fsm.update(obs, t)

        # ------------------------------------------------------------ M4 control + vehicle
        steer = accel = v_ref = 0.0
        if m4 is not None:
            act = m4.step(cmd, pose, v, dt)
            plant.step(act.steer, act.accel, act.direction, dt)
            steer, accel, v_ref = act.steer, act.accel, act.v_ref
            driven += abs(plant.car.v) * dt
            if t_first_move is None and abs(plant.car.v) > 0.05:
                t_first_move = t

        # ------------------------------------------------------------ log
        if m3_slot is not None:
            off = veh.length_m / 2 - veh.rear_overhang_m
            cx, cy = pose.x + off * math.cos(pose.yaw), pose.y + off * math.sin(pose.yaw)
            centre_err = math.hypot(cx - m3_slot.x, cy - m3_slot.y)
            head_err = math.degrees(abs(wrap(pose.yaw - m3_slot.yaw)))
        else:
            centre_err = head_err = float("nan")
        man = fsm.manoeuvre
        wr.writerow([f"{t:.3f}", fsm.state.value, cmd.mode.value, getattr(man, "phase", ""),
                     f"{pose.x:.4f}", f"{pose.y:.4f}", f"{pose.yaw:.4f}", f"{v:.4f}",
                     f"{steer:.4f}", f"{accel:.4f}", f"{v_ref:.4f}",
                     f"{m4.cte:.4f}" if m4 and cmd.mode.value == "follow_route" else "",
                     f"{clear['front']:.3f}", f"{clear['rear']:.3f}", f"{clear['left']:.3f}",
                     f"{clear['right']:.3f}", f"{min_clear:.3f}",
                     f"{centre_err:.4f}", f"{head_err:.3f}", getattr(man, "corrections", 0),
                     m3_slot.slot_id if m3_slot else "", cmd.reason])

        # ------------------------------------------------------------ end of task
        if fsm.state in (ParkingState.DONE, ParkingState.FAILED, ParkingState.EMERGENCY_STOP):
            if end_t is None:
                end_t = t
                print(f"[valet] {fsm.state.value.upper()}: {fsm.reason}", flush=True)
            if t - end_t >= CFG["idle_after_end_s"] and abs(v) < 0.01:
                break
    fh.close()

    summary = {
        "final_state": fsm.state.value, "reason": fsm.reason,
        "slot": m3_slot.slot_id if m3_slot else None,
        "task_time_s": None if t_first_move is None or end_t is None else round(end_t - t_first_move, 2),
        "distance_driven_m": round(driven, 2),
        "centre_error_cm": None if m3_slot is None else round(100 * centre_err, 1),
        "heading_error_deg": None if m3_slot is None else round(head_err, 2),
        "parked_check": bool(m3_slot and fsm.safety.parked(
            ParkingObservation(t, current_pose(), plant.car.v, 1, 1, 1, 1), m3_slot)),
        "min_clearance_m": round(min_clear_seen, 3),
        "corrections": getattr(fsm.manoeuvre, "corrections", 0),
        "lateral_controller": CFG["lateral"],
    }
    with open(os.path.join(CFG["log_dir"], f"webots_summary_{stamp}.json"), "w") as f:
        json.dump(summary, f, indent=2)
    print("[valet] SUMMARY " + json.dumps(summary), flush=True)
    try:
        robot.simulationSetMode(robot.SIMULATION_MODE_PAUSE)
    except Exception:
        pass


if __name__ == "__main__":
    main()
