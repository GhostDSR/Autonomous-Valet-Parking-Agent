"""
Run the whole valet agent offline (no Webots) and save figures + tables.

    python -m agent.run_offline              # the three scenarios below
    python -m agent.run_offline --sweep      # also: park in every bay, worst case

Scenarios (all start at the entrance with the default parked cars):
  normal      choose a slot, drive there, reverse into the bay
  slot_taken  another car takes our slot while we drive -> replan to a new slot
  pedestrian  a pedestrian steps onto the route ahead -> stop, wait / replan

Sweep: for each of the 40 bays, all other bays are occupied (the tightest
case: parked cars on both sides) and the agent must park in that one bay.
Gives the perpendicular-parking success rate per row.

The world is agent/sim_world.py (walls, parked cars, pedestrians, a
simulated 360-degree lidar), the car is M4's bicycle model with steering and
drivetrain lag. Everything else is the real M1 / M2 / M3 / M4 code.

Outputs go to results/agent/.
"""

import argparse
import csv
import math
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

from agent.sim_world import SimWorld
from agent.valet_agent import AgentConfig, ValetAgent
from m4_control.sim import Bicycle, VehicleParams as BicycleParams
from planner.compare_planners import (CRITICAL, INK, INK_MUTED, INK_SECONDARY, draw_lot,
                                      draw_pose)
from planner.parking_lot import DEFAULT_OCCUPIED, ENTRANCE_POSE, build_parking_lot, make_slots
from planner.vehicle import CollisionChecker

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_DIR = os.path.join(REPO_ROOT, "results", "agent")
DT = 0.032                                      # s, Webots basicTimeStep
PHASE_COLORS = {"SEARCH": INK_MUTED, "DRIVE": "#2a78d6", "STOPPING": CRITICAL,
                "ALIGN": "#eb6834", "PARK": "#1baf7a", "DONE": "#1baf7a",
                "FAILED": CRITICAL, "EMERGENCY_STOP": CRITICAL}

SCENARIOS = {
    "normal": "Drive from the entrance to the best free slot and park",
    "slot_taken": "Another car takes the chosen slot 3 s into the drive",
    "pedestrian": "A pedestrian steps onto the route 6 m ahead at 2 s and leaves at 12 s",
}


def body_gap(world, pose, geometry):
    """Smallest distance (m) between the car's outline and any obstacle (true geometry)."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    L, W, r = geometry.length_m, geometry.width_m, geometry.rear_overhang_m
    along = np.r_[np.linspace(-r, L - r, 24), np.full(10, -r), np.full(10, L - r)]
    across = np.r_[np.full(24, W / 2), np.linspace(-W / 2, W / 2, 10), np.linspace(-W / 2, W / 2, 10)]
    along, across = np.r_[along, along[:24]], np.r_[across, -across[:24]]
    px, py = x + along * c - across * s, y + along * s + across * c
    best = math.inf
    for cx, cy, bl, bw, byaw in world.boxes():
        bc, bs = math.cos(byaw), math.sin(byaw)
        u = np.abs(bc * (px - cx) + bs * (py - cy)) - bl / 2
        v = np.abs(-bs * (px - cx) + bc * (py - cy)) - bw / 2
        outside = np.hypot(np.maximum(u, 0), np.maximum(v, 0))
        inside = np.minimum(np.maximum(u, v), 0)          # negative when the point is inside
        best = min(best, float(np.min(outside + inside)))
    return best


def run(scenario="normal", occupied=DEFAULT_OCCUPIED, known_slots=None, t_max=180.0):
    """
    Simulate one run. known_slots (optional) = {slot id: "free"/"occupied"}
    tells the agent the slot states up front (as if M1 already scanned the lot).
    Returns a dict with the trace, events and final numbers.
    """
    world = SimWorld(occupied)
    agent = ValetAgent(config=AgentConfig(slots_from_lidar=known_slots is None))
    if known_slots:
        agent.slot_status = dict(known_slots)
    g = agent.geometry
    car = Bicycle(*ENTRANCE_POSE, BicycleParams(L=g.wheelbase_m, max_steer=g.max_steer_rad))

    trace, world_events = [], []
    min_gap, t, step = math.inf, 0.0, 0
    taken = ped_placed = ped_removed = False
    while t < t_max and agent.phase not in ("DONE", "FAILED", "EMERGENCY_STOP"):
        # --- things that happen in the world
        if scenario == "slot_taken" and not taken and t >= 3.0 and agent.choice:
            world.park_car(agent.choice.slot_id)
            world_events.append((t, f"another car parks in {agent.choice.slot_id}"))
            taken = True
        if scenario == "pedestrian" and not ped_placed and t >= 2.0 and agent.choice:
            i = agent.progress
            ahead = next((p for p in agent.choice.path[i:]
                          if math.hypot(p[0] - car.x, p[1] - car.y) >= 6.0), agent.choice.path[-1])
            world.pedestrians["p1"] = (ahead[0], ahead[1])
            world_events.append((t, f"pedestrian at ({ahead[0]:.1f}, {ahead[1]:.1f})"))
            ped_placed = True
        if scenario == "pedestrian" and ped_placed and not ped_removed and t >= 12.0:
            world.pedestrians.pop("p1")
            world_events.append((t, "pedestrian leaves"))
            ped_removed = True

        # --- one control step: sense -> agent -> car
        pose = (car.x, car.y, car.yaw)
        act = agent.step(t, pose, car.v, world.lidar_scan(pose))
        car.step(act.steer, act.accel, act.direction, DT)
        trace.append((t, car.x, car.y, car.yaw, car.v, agent.phase))
        if step % 5 == 0:
            min_gap = min(min_gap, body_gap(world, (car.x, car.y, car.yaw), g))
        t += DT
        step += 1

    result = {"scenario": scenario, "phase": agent.phase, "reason": agent.reason,
              "time_s": t, "trace": trace, "events": agent.events, "world_events": world_events,
              "plans": agent.plans, "slot": agent.choice.slot_id if agent.choice else None,
              "replans": max(0, len(agent.plans) - 1), "min_gap_m": min_gap,
              "occupied": set(world.occupied), "pedestrians": dict(world.pedestrians),
              "centre_error_cm": None, "heading_error_deg": None}
    if agent.choice:
        s = agent.slots[agent.choice.slot_id]
        off = g.length_m / 2 - g.rear_overhang_m
        cx, cy = car.x + off * math.cos(car.yaw), car.y + off * math.sin(car.yaw)
        result["centre_error_cm"] = 100 * math.hypot(cx - s.x, cy - s.y)
        result["heading_error_deg"] = abs(math.degrees((car.yaw - s.yaw + math.pi) % (2 * math.pi) - math.pi))
    return result


# ---------------------------------------------------------------- outputs

def plot_run(res, path):
    grid_map, slots = build_parking_lot(occupied=res["occupied"])
    checker = CollisionChecker(grid_map, ValetAgent().vehicle)
    fig, ax = plt.subplots(figsize=(10, 8.5))
    draw_lot(ax, grid_map, checker, slots, target=res["slot"])

    for i, (_, sid, plan) in enumerate(res["plans"]):
        ax.plot([p[0] for p in plan], [p[1] for p in plan], color=INK_SECONDARY, lw=1,
                ls=":" if i < len(res["plans"]) - 1 else "--", zorder=3)
    tr = res["trace"]
    for phase, color in PHASE_COLORS.items():
        pts = [(r[1], r[2]) for r in tr if r[5] == phase]
        if pts:
            ax.plot(*zip(*pts), ".", color=color, markersize=2.5, zorder=4)
    for name, (px, py) in {**res["pedestrians"], **_peds_seen(res)}.items():
        ax.plot(px, py, "o", color=CRITICAL, markersize=8, zorder=6)
    draw_pose(ax, ENTRANCE_POSE, "start", "s")

    handles = [Line2D([], [], color=c, marker="o", ls="", label=p.lower().replace("_", " "))
               for p, c in PHASE_COLORS.items() if any(r[5] == p for r in tr) and p != "DONE"]
    handles += [Line2D([], [], color=INK_SECONDARY, ls="--", label="planned route (M2)")]
    if len(res["plans"]) > 1:
        handles += [Line2D([], [], color=INK_SECONDARY, ls=":", label="abandoned route")]
    if res["world_events"] and res["scenario"] == "pedestrian":
        handles += [Line2D([], [], color=CRITICAL, marker="o", ls="", label="pedestrian")]
    ax.legend(handles=handles, loc="upper right", fontsize=8, framealpha=0.95)
    ax.set_title(f"{res['scenario']}: {res['phase'].lower()} in {res['slot']} after "
                 f"{res['time_s']:.0f} s, {res['replans']} replan(s)", color=INK, loc="left")
    fig.savefig(path, dpi=130, bbox_inches="tight")
    plt.close(fig)


def _peds_seen(res):
    """Pedestrian positions from the world events (they may have left by the end)."""
    out = {}
    for _, text in res["world_events"]:
        if text.startswith("pedestrian at"):
            x, y = text[text.index("(") + 1:text.index(")")].split(",")
            out["p1"] = (float(x), float(y))
    return out


def write_table(rows, path_md, path_csv, title):
    fields = ["scenario", "slot", "phase", "time_s", "replans", "min_gap_m",
              "centre_error_cm", "heading_error_deg"]
    with open(path_csv, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(fields)
        for r in rows:
            w.writerow([_fmt(r[f]) for f in fields])
    with open(path_md, "w") as fh:
        fh.write(f"# {title}\n\nGenerated by `python -m agent.run_offline`.\n\n")
        fh.write("| Scenario | Slot | Result | Time (s) | Replans | Closest gap (m) | "
                 "Centre error (cm) | Heading error (deg) |\n|---|---|---|---|---|---|---|---|\n")
        for r in rows:
            fh.write("| " + " | ".join(_fmt(r[f]) for f in fields) + " |\n")


def _fmt(v):
    if v is None:
        return "-"
    if isinstance(v, float):
        return f"{v:.2f}"
    return str(v)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--sweep", action="store_true", help="also park in every bay (slow)")
    args = ap.parse_args()
    os.makedirs(OUT_DIR, exist_ok=True)

    rows = []
    for name, desc in SCENARIOS.items():
        res = run(name)
        rows.append(res)
        plot_run(res, os.path.join(OUT_DIR, f"agent_{name}.png"))
        print(f"\n== {name}: {desc}")
        for t, text in sorted(res["world_events"] + res["events"]):
            print(f"  {t:6.2f} s  {text}")
        print(f"  result: {res['phase']} in {res['slot']}, closest gap {res['min_gap_m']:.2f} m")
    write_table(rows, os.path.join(OUT_DIR, "agent_scenarios.md"),
                os.path.join(OUT_DIR, "agent_scenarios.csv"), "Valet agent: offline scenarios")

    if args.sweep:
        slots = make_slots()
        sweep = []
        for sid in sorted(slots):
            occupied = set(slots) - {sid}
            known = {s: ("occupied" if s in occupied else "free") for s in slots}
            res = run("sweep", occupied, known)
            res["scenario"] = f"only {sid} free"
            sweep.append(res)
            print(f"{sid}: {res['phase']:15s} gap {res['min_gap_m']:.2f} m  {res['reason']}", flush=True)
        write_table(sweep, os.path.join(OUT_DIR, "agent_sweep.md"),
                    os.path.join(OUT_DIR, "agent_sweep.csv"),
                    "Valet agent: park in every bay, both neighbours occupied")
        with open(os.path.join(OUT_DIR, "agent_sweep.md"), "a") as fh:
            fh.write("\n## Success rate per row\n\n| Row | Parked | Rate |\n|---|---|---|\n")
            for row in "ABCD":
                rr = [r for r in sweep if r["scenario"].split()[1].startswith(row)]
                ok = sum(r["phase"] == "DONE" for r in rr)
                fh.write(f"| {row} | {ok} / {len(rr)} | {100 * ok / len(rr):.0f}% |\n")
    print("\noutputs ->", OUT_DIR)


if __name__ == "__main__":
    main()
