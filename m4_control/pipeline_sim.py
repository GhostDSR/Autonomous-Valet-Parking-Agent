"""Whole pipeline offline (no Webots): M2-style route -> M3 state machine -> M4 -> vehicle model.

    python -m m4_control.pipeline_sim                  # normal run
    python -m m4_control.pipeline_sim --obstacle       # an obstacle appears -> emergency stop test

Uses the REAL M3 code (behaviours/) and M4; only the route, the slot and the vehicle
are synthetic stand-ins (replace with M2 / M1 data in Webots).
"""
import argparse
import math
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

from behaviours.parking_fsm import ParkingFSM
from behaviours.parking_manoeuvre import ParkingManoeuvre
from behaviours.parking_types import (ParkingObservation, ParkingSlot, ParkingState, Pose,
                                      VehicleGeometry)
from . import run_experiments as _re
from .follower import FollowerConfig
from .lateral import Stanley
from .m4_controller import M4Controller
from .path import Path, fillet_polyline
from .sim import Bicycle, VehicleParams

ap = argparse.ArgumentParser()
ap.add_argument("--obstacle", action="store_true", help="obstacle appears 8 s into the route")
a = ap.parse_args()

# ---- stand-ins for M1/M2 data (planner/vehicle.py placeholders) -----------------
VEH = VehicleGeometry(wheelbase_m=2.8, length_m=4.6, width_m=1.9, rear_overhang_m=1.0, max_steer_rad=0.5)
SLOT = ParkingSlot("T1", x=20.0, y=5.0, yaw=math.pi / 2, width_m=2.5, depth_m=5.0)
PRE = Pose(23.0, 11.0, 0.0)                       # aisle centre line, 3 m past the bay
route_xy = fillet_polyline([[2, 8.5], [8, 8.5], [10.5, 11], [23, 11]], 5.0)
route = Path.from_points(route_xy)

vp = VehicleParams(L=VEH.wheelbase_m, max_steer=VEH.max_steer_rad)
car = Bicycle(2.0, 8.5, 0.0, vp)
fcfg = FollowerConfig(max_steer=VEH.max_steer_rad, v_fwd=1.5)   # 1.5 m/s: M3's safety needs
                                                                # clearance > stopping distance
m4 = M4Controller(route, Stanley(L=VEH.wheelbase_m, use_ff=True), VEH.max_steer_rad, fcfg)
fsm = ParkingFSM(VEH, ParkingManoeuvre(VEH))
fsm.select_slot(SLOT, PRE)

dt, t = 0.032, 0.0                                  # typical Webots basicTimeStep
trace, states = [], []
while t < 200 and fsm.state not in (ParkingState.DONE, ParkingState.FAILED, ParkingState.EMERGENCY_STOP):
    front = 0.1 if (a.obstacle and t > 8.0) else 6.0
    obs = ParkingObservation(timestamp_s=t, pose=Pose(car.x, car.y, car.yaw), speed_mps=car.v,
                             front_clearance_m=front, rear_clearance_m=6.0,
                             left_clearance_m=3.0, right_clearance_m=3.0,
                             route_complete=m4.route_complete)
    cmd = fsm.update(obs, t)
    act = m4.step(cmd, obs.pose, car.v, dt)
    car.step(act.steer, act.accel, act.direction, dt)
    trace.append((t, car.x, car.y, car.yaw, car.v))
    states.append(fsm.state.value)
    t += dt

# let a latched stop finish braking
for _ in range(200):
    obs = ParkingObservation(t, Pose(car.x, car.y, car.yaw), car.v, 6.0, 6.0, 3.0, 3.0, m4.route_complete)
    cmd = fsm.update(obs, t)
    act = m4.step(cmd, obs.pose, car.v, dt)
    car.step(act.steer, act.accel, act.direction, dt)
    trace.append((t, car.x, car.y, car.yaw, car.v)); states.append(fsm.state.value)
    t += dt

tr = np.array(trace)
print(f"final state : {fsm.state.value}   ({fsm.reason})")
print(f"time        : {tr[-1, 0]:.1f} s   final speed: {car.v:.3f} m/s")
if fsm.state == ParkingState.DONE:
    print(f"parked check: {fsm.safety.parked(ParkingObservation(t, Pose(car.x, car.y, car.yaw), car.v, 6, 6, 3, 3), SLOT)}")
    ang = math.degrees(abs((car.yaw - SLOT.yaw + math.pi) % (2 * math.pi) - math.pi))
    cx = car.x + (VEH.length_m / 2 - VEH.rear_overhang_m) * math.cos(car.yaw)
    cy = car.y + (VEH.length_m / 2 - VEH.rear_overhang_m) * math.sin(car.yaw)
    print(f"final pose  : centre error {100*math.hypot(cx-SLOT.x, cy-SLOT.y):.1f} cm, heading error {ang:.2f} deg")
seq = [s for i, s in enumerate(states) if i == 0 or s != states[i - 1]]
print("state order :", " -> ".join(seq))

# ---- figure ----------------------------------------------------------------------
fig, ax = plt.subplots(figsize=(11, 7))
sl = np.array([[-SLOT.depth_m / 2, -SLOT.width_m / 2], [SLOT.depth_m / 2, -SLOT.width_m / 2],
               [SLOT.depth_m / 2, SLOT.width_m / 2], [-SLOT.depth_m / 2, SLOT.width_m / 2]])
R = np.array([[math.cos(SLOT.yaw), -math.sin(SLOT.yaw)], [math.sin(SLOT.yaw), math.cos(SLOT.yaw)]])
ax.add_patch(Polygon(sl @ R.T + [SLOT.x, SLOT.y], fill=False, ec="gray", lw=2, label="bay"))
ax.plot(route_xy[:, 0], route_xy[:, 1], "k--", lw=1, label="M2 route")
sc = ax.scatter(tr[:, 1], tr[:, 2], c=tr[:, 4], cmap="coolwarm", s=4, vmin=-0.8, vmax=1.6)
plt.colorbar(sc, label="speed [m/s]  (blue = reversing)")
c, s = math.cos(car.yaw), math.sin(car.yaw)
body = np.array([[-1.0, -0.95], [3.6, -0.95], [3.6, 0.95], [-1.0, 0.95]])
ax.add_patch(Polygon(body @ np.array([[c, s], [-s, c]]) + [car.x, car.y], fill=False, ec="C2", lw=2, label="final car"))
ax.plot(PRE.x, PRE.y, "r*", ms=12, label="pre-park pose")
ax.set_aspect("equal"); ax.legend(loc="upper left"); ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
ax.set_title("Full pipeline: M2 route -> M3 state machine -> M4 control" + (" (obstacle test)" if a.obstacle else ""))
out = os.path.join(_re.OUT, "figures"); os.makedirs(out, exist_ok=True)
name = "F8_pipeline_obstacle.png" if a.obstacle else "F8_pipeline.png"
fig.savefig(os.path.join(out, name), dpi=130, bbox_inches="tight")
print("figure ->", os.path.join(out, name))
