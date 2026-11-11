"""Adapter: M2 planner output  ->  M4 Path.

    from planner import build_parking_lot, plan_to_slot
    from m4_control.planner_bridge import path_from_planner

    path, stats, goal = plan_to_slot(grid_map, start, slots["C4"])
    m4_path = path_from_planner(path, stats["directions"])
"""
import numpy as np
from .path import Path


def path_from_planner(path, directions=None):
    """path: list of (x, y, heading); directions: +1 forward / -1 reverse per pose."""
    xy = np.array([[p[0], p[1]] for p in path], float)
    if len(xy) < 2:
        raise ValueError("planner returned a path with fewer than 2 poses")
    d = np.ones(len(xy)) if directions is None else np.asarray(directions, float)
    if len(d) < len(xy):                                   # tolerate len(path)-1 directions
        d = np.r_[d, np.full(len(xy) - len(d), d[-1])]
    d = np.where(d[:len(xy)] >= 0, 1, -1)
    for i in range(1, len(d) - 1):                         # drop 1-pose direction flickers
        if d[i] != d[i - 1] and d[i] != d[i + 1]:
            d[i] = d[i - 1]
    if len(d) > 1 and d[0] != d[1]:
        d[0] = d[1]
    return Path.from_points(xy, d)
