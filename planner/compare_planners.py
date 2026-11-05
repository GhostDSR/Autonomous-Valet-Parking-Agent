"""
Experiment: compare A*, A* + smoothing and Hybrid A* on the parking lot.

Part of M2 - Route Planner. Produces the numbers and figures for the Results
and Novelty sections of the report.

Run from the repository root:

    python -m planner.compare_planners            # default: 3 timing repeats
    python -m planner.compare_planners --repeats 5

For each target slot, the goal pose is fixed first (the pre-park pose
Hybrid A* picks for that slot), and then ALL methods plan to that same goal
from the entrance. That keeps the comparison fair.

Outputs (in results/):
    planner_comparison.csv       one row per (slot, method)
    planner_comparison.md        summary + per-slot tables (Markdown)
    planner_paths.png            all routes, one panel per slot
    planner_paths_<slot>.png     the same, one figure per slot
    planner_curvature_<slot>.png curvature along the path, showcase slot
    planner_summary.png          bar charts of the averaged metrics
"""

import argparse
import csv
import math
import os
import statistics

import matplotlib

matplotlib.use("Agg")  # write files only, never open a window
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.colors import ListedColormap  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402

from planner.metrics import compute_metrics, curvature_profile, split_by_direction  # noqa: E402
from planner.parking_lot import (BAY_DEPTH, BAY_WIDTH, ENTRANCE_POSE,  # noqa: E402
                                 build_parking_lot)
from planner.route_planner import METHODS, plan_route_with_stats, plan_to_slot  # noqa: E402
from planner.vehicle import CollisionChecker, VehicleParams  # noqa: E402

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RESULTS_DIR = os.path.join(REPO_ROOT, "results")

# Free slots used as targets: two easy (same aisle as the entrance) and four
# that need turns into the second aisle or the side lane.
SCENARIOS = ["A5", "B3", "C4", "C9", "D0", "D6"]
SHOWCASE = "C4"

LABELS = {"astar": "A*", "astar_smoothed": "A* + smoothing", "hybrid_astar": "Hybrid A*"}

# Colours: categorical slots 1-3 of the reference palette (validated as a set
# of three for colour-blind safety), plus neutral inks for the map.
COLORS = {"astar": "#2a78d6", "astar_smoothed": "#eb6834", "hybrid_astar": "#1baf7a"}
INK = "#0b0b0b"
INK_SECONDARY = "#52514e"
INK_MUTED = "#898781"
GRIDLINE = "#e1e0d9"
SURFACE = "#fcfcfb"
OBSTACLE = "#52514e"
MARGIN = "#e1e0d9"
CRITICAL = "#d03b3b"

plt.rcParams.update({
    "font.family": "sans-serif",
    "font.size": 10,
    "axes.edgecolor": "#c3c2b7",
    "axes.labelcolor": INK_SECONDARY,
    "xtick.color": INK_MUTED,
    "ytick.color": INK_MUTED,
    "axes.titlecolor": INK,
    "figure.facecolor": SURFACE,
    "axes.facecolor": SURFACE,
    "savefig.facecolor": SURFACE,
})


# ---------------------------------------------------------------------------
# Running the experiment
# ---------------------------------------------------------------------------

def run_experiment(repeats):
    grid_map, slots = build_parking_lot()
    vehicle = VehicleParams()
    checker = CollisionChecker(grid_map, vehicle)  # built once, shared by all methods

    rows, paths = [], {}
    for sid in SCENARIOS:
        # 1. Fix the goal: the pre-park pose Hybrid A* chooses for this slot.
        _, _, goal = plan_to_slot(grid_map, ENTRANCE_POSE, slots[sid], "hybrid_astar",
                                  vehicle, checker=checker)
        if goal is None:
            print(f"{sid}: no reachable pre-park pose, skipped")
            continue

        # 2. Every method plans from the entrance to that same goal.
        for method in METHODS:
            times = []
            for _ in range(repeats):
                path, stats = plan_route_with_stats(grid_map, ENTRANCE_POSE, goal, method,
                                                    vehicle, checker=checker)
                times.append(stats["time_s"])
            m = compute_metrics(path, vehicle, checker, goal, stats["directions"])
            paths[(sid, method)] = (path, stats["directions"], goal)
            row = {
                "slot": sid,
                "method": method,
                "found": m["found"],
                "time_ms": 1000 * statistics.median(times),
                "nodes_expanded": stats["nodes_expanded"],
            }
            if m["found"]:
                row.update({k: m[k] for k in (
                    "length_m", "max_curvature", "over_limit_m", "total_turning_deg",
                    "direction_switches", "footprint_hits", "goal_pos_error_m",
                    "goal_yaw_error_deg")})
            rows.append(row)
            print(f"{sid:>3} {LABELS[method]:<15} found={m['found']}  "
                  f"time={row['time_ms']:8.1f} ms  length={row.get('length_m', float('nan')):5.1f} m  "
                  f"max curv={row.get('max_curvature', float('nan')):.2f}  "
                  f"footprint hits={row.get('footprint_hits', '-')}")
    return grid_map, slots, vehicle, checker, rows, paths


def summarise(rows):
    """
    Per method, over all slots where a path was found: the mean of each
    metric, except max curvature, which is the worst (largest) over all routes.
    """
    keys = ["length_m", "time_ms", "nodes_expanded", "over_limit_m",
            "total_turning_deg", "direction_switches", "footprint_hits", "goal_yaw_error_deg"]
    summary = {}
    for method in METHODS:
        found = [r for r in rows if r["method"] == method and r["found"]]
        total = [r for r in rows if r["method"] == method]
        summary[method] = {k: statistics.mean(r[k] for r in found) for k in keys} if found else {}
        if found:
            summary[method]["max_curvature"] = max(r["max_curvature"] for r in found)
        summary[method]["success"] = f"{len(found)}/{len(total)}"
    return summary


# ---------------------------------------------------------------------------
# Tables
# ---------------------------------------------------------------------------

CSV_FIELDS = ["slot", "method", "found", "length_m", "time_ms", "nodes_expanded",
              "max_curvature", "over_limit_m", "total_turning_deg", "direction_switches",
              "footprint_hits", "goal_pos_error_m", "goal_yaw_error_deg"]


def write_csv(rows, path):
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for r in rows:
            writer.writerow({k: (round(v, 4) if isinstance(v, float) else v)
                             for k, v in r.items()})


def write_markdown(rows, summary, vehicle, repeats, path):
    lines = [
        "# Planner comparison: A* vs A* + smoothing vs Hybrid A*",
        "",
        "Generated by `python -m planner.compare_planners`. Start: entrance pose "
        f"{ENTRANCE_POSE}. Goal: the pre-park pose of each slot (same goal for every method).",
        f"Car: wheelbase {vehicle.wheelbase} m, max steer {math.degrees(vehicle.max_steer):.1f} deg, "
        f"min turning radius {vehicle.min_turn_radius:.2f} m, "
        f"max curvature {vehicle.max_curvature:.3f} 1/m. "
        f"Planning time is the median of {repeats} runs.",
        "",
        "## Mean over all slots",
        "",
        "(Max curvature is the worst route, not the mean.)",
        "",
        "| Method | Success | Length (m) | Time (ms) | Nodes expanded | Max curvature, worst route (1/m) "
        "| Over-limit length (m) | Total turning (deg) | Footprint hits | Goal heading error (deg) |",
        "|---|---|---|---|---|---|---|---|---|---|",
    ]
    for method in METHODS:
        s = summary[method]
        if "length_m" not in s:
            lines.append(f"| {LABELS[method]} | {s['success']} | - | - | - | - | - | - | - | - |")
            continue
        lines.append(
            f"| {LABELS[method]} | {s['success']} | {s['length_m']:.1f} | {s['time_ms']:.1f} "
            f"| {s['nodes_expanded']:.0f} | {s['max_curvature']:.2f} | {s['over_limit_m']:.2f} "
            f"| {s['total_turning_deg']:.0f} | {s['footprint_hits']:.1f} "
            f"| {s['goal_yaw_error_deg']:.1f} |")

    lines += [
        "",
        "## Per slot",
        "",
        "| Slot | Method | Length (m) | Time (ms) | Max curvature (1/m) | Over-limit length (m) "
        "| Footprint hits | Direction switches | Goal heading error (deg) |",
        "|---|---|---|---|---|---|---|---|---|",
    ]
    for r in rows:
        if not r["found"]:
            lines.append(f"| {r['slot']} | {LABELS[r['method']]} | no path | {r['time_ms']:.1f} "
                         "| - | - | - | - | - |")
            continue
        lines.append(
            f"| {r['slot']} | {LABELS[r['method']]} | {r['length_m']:.1f} | {r['time_ms']:.1f} "
            f"| {r['max_curvature']:.2f} | {r['over_limit_m']:.2f} | {r['footprint_hits']} "
            f"| {r['direction_switches']} | {r['goal_yaw_error_deg']:.1f} |")

    lines += [
        "",
        "## How to read the columns",
        "",
        "- **Max curvature**: sharpest bend on the path. Above the car's max curvature "
        f"({vehicle.max_curvature:.3f} 1/m) the car cannot follow the path.",
        "- **Over-limit length**: metres of path that bend more sharply than the car can steer.",
        "- **Footprint hits**: path poses where the full car rectangle would touch an obstacle "
        "(A* only checks a disc around the rear axle).",
        "- **Goal heading error**: difference between the car's final heading and the heading "
        "the parking manoeuvre needs. Grid A* has no notion of heading.",
        "",
    ]
    with open(path, "w") as f:
        f.write("\n".join(lines))


# ---------------------------------------------------------------------------
# Figures
# ---------------------------------------------------------------------------

def draw_lot(ax, grid_map, checker, slots, target=None):
    """Lot background: obstacles (dark), inflation margin (light), bay lines, target bay."""
    layer = checker.inflated.data.astype(int)
    layer[grid_map.data == 1] = 2
    extent = (grid_map.origin_x, grid_map.origin_x + grid_map.width_m,
              grid_map.origin_y, grid_map.origin_y + grid_map.height_m)
    ax.imshow(layer, cmap=ListedColormap([SURFACE, MARGIN, OBSTACLE]), vmin=0, vmax=2,
              origin="lower", extent=extent, interpolation="nearest")

    for s in slots.values():
        is_target = s.slot_id == target
        ax.add_patch(Rectangle((s.x - BAY_WIDTH / 2, s.y - BAY_DEPTH / 2), BAY_WIDTH, BAY_DEPTH,
                               fill=False, lw=1.6 if is_target else 0.4,
                               edgecolor=INK if is_target else "#c3c2b7", zorder=2))
    if target:
        s = slots[target]
        ax.text(s.x, s.y, target, ha="center", va="center", fontsize=8, color=INK,
                fontweight="bold", zorder=3)

    ax.set_xlim(extent[0], extent[1])
    ax.set_ylim(extent[2], extent[3])
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")


def draw_pose(ax, pose, label, marker):
    x, y, yaw = pose
    ax.plot(x, y, marker, color=INK, markersize=7, zorder=6)
    ax.annotate("", xy=(x + 2.0 * math.cos(yaw), y + 2.0 * math.sin(yaw)), xytext=(x, y),
                arrowprops=dict(arrowstyle="-|>", color=INK, lw=1.2), zorder=6)
    ax.annotate(label, (x, y), textcoords="offset points", xytext=(4, -12),
                fontsize=8, color=INK_SECONDARY, zorder=6)


def draw_paths(ax, sid, paths, checker):
    for method in METHODS:
        path, directions, _ = paths[(sid, method)]
        if not path:
            continue
        color = COLORS[method]
        # Forward stretches solid, reverse stretches dashed
        dirs = directions or [1] * len(path)
        pieces = split_by_direction(path, dirs)
        idx = 0
        for piece in pieces:
            d = dirs[min(idx + 1, len(dirs) - 1)]
            ax.plot([p[0] for p in piece], [p[1] for p in piece], color=color, lw=2,
                    ls="-" if d > 0 else "--", zorder=4, solid_capstyle="round")
            idx += len(piece) - 1
        hits = [p for p in path if checker.pose_collides(*p)]
        if hits:
            ax.plot([p[0] for p in hits], [p[1] for p in hits], "x", color=CRITICAL,
                    markersize=7, mew=2, zorder=5)
    draw_pose(ax, ENTRANCE_POSE, "start", "s")
    draw_pose(ax, paths[(sid, METHODS[0])][2], "goal", "o")


def legend_handles():
    handles = [Line2D([], [], color=COLORS[m], lw=2, label=LABELS[m]) for m in METHODS]
    handles += [
        Line2D([], [], color=INK_SECONDARY, lw=2, ls="--", label="reversing"),
        Line2D([], [], color=CRITICAL, marker="x", ls="", mew=2, label="car body hits obstacle"),
        Rectangle((0, 0), 1, 1, color=OBSTACLE, label="obstacle / parked car"),
        Rectangle((0, 0), 1, 1, color=MARGIN, label="safety margin (inflation)"),
    ]
    return handles


def plot_all_paths(grid_map, checker, slots, paths, out_dir):
    sids = [s for s in SCENARIOS if (s, METHODS[0]) in paths]
    ncols = 3
    nrows = math.ceil(len(sids) / ncols)
    fig, axes = plt.subplots(nrows, ncols, figsize=(15, 4.6 * nrows + 0.8))
    for ax in axes.flat[len(sids):]:
        ax.axis("off")
    for ax, sid in zip(axes.flat, sids):
        draw_lot(ax, grid_map, checker, slots, sid)
        draw_paths(ax, sid, paths, checker)
        ax.set_title(f"Route to slot {sid}", loc="left", fontsize=11)
    fig.legend(handles=legend_handles(), loc="upper center", ncol=7, frameon=False,
               fontsize=9, bbox_to_anchor=(0.5, 1.0))
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(os.path.join(out_dir, "planner_paths.png"), dpi=150)
    plt.close(fig)

    for sid in sids:
        fig, ax = plt.subplots(figsize=(9, 8))
        draw_lot(ax, grid_map, checker, slots, sid)
        draw_paths(ax, sid, paths, checker)
        ax.set_title(f"Route from entrance to slot {sid}", loc="left", fontsize=12)
        ax.legend(handles=legend_handles(), loc="upper center", bbox_to_anchor=(0.5, -0.08),
                  ncol=4, frameon=False, fontsize=9)
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, f"planner_paths_{sid}.png"), dpi=150)
        plt.close(fig)


def plot_curvature(sid, paths, vehicle, out_dir):
    fig, ax = plt.subplots(figsize=(9, 4))
    s_max = 0.0
    for method in METHODS:
        path, directions, _ = paths[(sid, method)]
        if not path:
            continue
        s_all, k_all, offset = [], [], 0.0
        for piece in split_by_direction([(p[0], p[1]) for p in path], directions):
            s, k = curvature_profile(piece)
            s_all += [offset + v for v in s]
            k_all += k
            offset += sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(piece, piece[1:]))
        ax.plot(s_all, k_all, color=COLORS[method], lw=2, label=LABELS[method])
        s_max = max(s_max, offset)
    limit = vehicle.max_curvature
    ax.axhline(limit, color=INK_MUTED, lw=1, ls="--")
    ax.annotate(f"car limit 1/R_min = {limit:.2f} 1/m", (s_max, limit), textcoords="offset points",
                xytext=(0, 4), fontsize=8, color=INK_SECONDARY, ha="right")
    ax.set_xlabel("distance along path (m)")
    ax.set_ylabel("curvature (1/m)")
    ax.set_title(f"Path curvature, route to slot {sid}", loc="left", fontsize=12)
    ax.grid(axis="y", color=GRIDLINE, lw=0.6)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    ax.legend(frameon=False)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, f"planner_curvature_{sid}.png"), dpi=150)
    plt.close(fig)


def plot_summary(summary, vehicle, out_dir):
    panels = [
        ("length_m", "Path length (m)", "{:.1f}"),
        ("time_ms", "Planning time (ms)", "{:.0f}"),
        ("max_curvature", "Max curvature, worst route (1/m)", "{:.2f}"),
        ("over_limit_m", "Path the car can't follow (m)", "{:.2f}"),
        ("footprint_hits", "Footprint collisions (poses)", "{:.1f}"),
        ("goal_yaw_error_deg", "Final heading error (deg)", "{:.0f}"),
    ]
    methods = [m for m in METHODS if "length_m" in summary[m]]
    fig, axes = plt.subplots(2, 3, figsize=(13, 6.5))
    for ax, (key, title, fmt) in zip(axes.flat, panels):
        values = [summary[m][key] for m in methods]
        xs = range(len(methods))
        ax.bar(xs, values, width=0.6, color=[COLORS[m] for m in methods], zorder=3)
        top = max(values) if max(values) > 0 else 1.0
        for x, v in zip(xs, values):
            ax.text(x, v + 0.02 * top, fmt.format(v), ha="center", va="bottom",
                    fontsize=9, color=INK)
        if key == "max_curvature":
            ax.axhline(vehicle.max_curvature, color=INK_MUTED, lw=1, ls="--", zorder=4)
            # Label sits in the gap between the 2nd and 3rd bars, above the line
            ax.annotate("car limit", (1.5, vehicle.max_curvature),
                        textcoords="offset points", xytext=(0, 3), fontsize=8,
                        color=INK_SECONDARY, ha="center", zorder=5)
        ax.set_xticks(list(xs))
        ax.set_xticklabels([LABELS[m] for m in methods])
        ax.set_ylim(0, top * 1.15)
        ax.set_title(title, loc="left", fontsize=11)
        ax.grid(axis="y", color=GRIDLINE, lw=0.6)
        ax.set_axisbelow(True)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
    fig.suptitle(f"Planner comparison over {len(SCENARIOS)} routes (mean, except max curvature)",
                 x=0.01, ha="left",
                 fontsize=13, color=INK)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "planner_summary.png"), dpi=150)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[1])
    parser.add_argument("--repeats", type=int, default=3, help="timing repeats per method")
    args = parser.parse_args()

    os.makedirs(RESULTS_DIR, exist_ok=True)
    grid_map, slots, vehicle, checker, rows, paths = run_experiment(args.repeats)
    summary = summarise(rows)

    write_csv(rows, os.path.join(RESULTS_DIR, "planner_comparison.csv"))
    write_markdown(rows, summary, vehicle, args.repeats,
                   os.path.join(RESULTS_DIR, "planner_comparison.md"))
    plot_all_paths(grid_map, checker, slots, paths, RESULTS_DIR)
    if (SHOWCASE, METHODS[0]) in paths:
        plot_curvature(SHOWCASE, paths, vehicle, RESULTS_DIR)
    plot_summary(summary, vehicle, RESULTS_DIR)
    print(f"Wrote tables and figures to {RESULTS_DIR}")


if __name__ == "__main__":
    main()
