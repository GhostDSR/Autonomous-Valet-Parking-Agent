"""Run M4 on the REAL M2 planner output (offline, no Webots) and save a plot.

From the repo root:
    python -m m4_control.run_on_planner                 # default: slot C4, occupied A0 A1 B2
    python -m m4_control.run_on_planner --slot D7 --occupied A0 B3 C1
M4 tracks the route to the pre-park pose and stops there; M3 then reverses into the bay.
"""
import argparse
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from planner import build_parking_lot, plan_to_slot
from . import run_experiments as _re
from .lateral import PurePursuit, Stanley
from .planner_bridge import path_from_planner
from .sim import VehicleParams, run_episode
from .follower import FollowerConfig

ap = argparse.ArgumentParser()
ap.add_argument("--slot", default="C4")
ap.add_argument("--occupied", nargs="*", default=["A0", "A1", "B2"])
ap.add_argument("--start", nargs=3, type=float, default=[2.0, 8.5, 0.0], metavar=("X", "Y", "YAW"))
ap.add_argument("--wheelbase", type=float, default=2.8)     # planner/vehicle.py placeholder
ap.add_argument("--max-steer", type=float, default=0.5)
a = ap.parse_args()

grid_map, slots = build_parking_lot(occupied=set(a.occupied))
plan, stats, goal = plan_to_slot(grid_map, tuple(a.start), slots[a.slot])
if not plan:
    raise SystemExit("planner found no route")
path = path_from_planner(plan, stats.get("directions"))
print(f"planned {len(plan)} poses, {len(path.segments)} segment(s), "
      f"{sum(s.s_end for s in path.segments):.1f} m, goal = {goal}")

L, ms = a.wheelbase, a.max_steer
vp = VehicleParams(L=L, max_steer=ms)
cfg = FollowerConfig(max_steer=ms)
out = os.path.join(_re.OUT, "figures")
os.makedirs(out, exist_ok=True)

fig, ax = plt.subplots(figsize=(11, 9))
ax.plot([p[0] for p in plan], [p[1] for p in plan], "k--", lw=1.2, label="planned (M2)")
for name, lat in {"Pure Pursuit": PurePursuit(L=L), "Stanley+FF": Stanley(L=L, use_ff=True)}.items():
    log = run_episode(path, lat, cfg=cfg, vp=vp, init_offset=(0.0, 0.0))
    cte = np.abs(log["cte"])
    gx, gy, gyaw = goal
    d_goal = np.hypot(log["x"][-1] - gx, log["y"][-1] - gy)
    d_yaw = np.degrees(abs((log["yaw"][-1] - gyaw + np.pi) % (2 * np.pi) - np.pi))
    print(f"{name:13s} done={log['done']}  RMS CTE={np.sqrt(np.mean(cte**2)):.3f} m  "
          f"max CTE={cte.max():.3f} m  end-to-goal={d_goal*100:.1f} cm / {d_yaw:.1f} deg  "
          f"time={log['t'][-1]:.1f} s")
    ax.plot(log["x"], log["y"], lw=1.5, label=name)
ax.plot(*goal[:2], "r*", ms=14, label="pre-park goal")
ax.set_aspect("equal"); ax.legend(); ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
ax.set_title(f"M4 tracking the M2 route to slot {a.slot}")
fig.savefig(os.path.join(out, "F7_planner_route_tracking.png"), dpi=130, bbox_inches="tight")
print("figure ->", os.path.join(out, "F7_planner_route_tracking.png"))
