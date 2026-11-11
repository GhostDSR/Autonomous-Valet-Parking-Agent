"""
Single entry point to the route planner for the rest of the team.

Part of M2 - Route Planner.

Typical use from the Webots controller:

    from planner import build_parking_lot, plan_to_slot, ENTRANCE_POSE

    grid_map, slots = build_parking_lot(occupied={"A0", "B1", ...})
    path, stats, goal = plan_to_slot(grid_map, current_pose, slots["C4"])
    # path = [(x, y, heading), ...] in Webots metres / radians (rear axle)
    # stats["directions"] = +1 / -1 per pose (drive forward / reverse)

Three methods are available, all returning the same format:
    "astar"           grid A* (car treated as a disc), headings from segments
    "astar_smoothed"  grid A* + gradient smoothing
    "hybrid_astar"    Hybrid A* (respects the turning radius)  <- default
"""

import math
import time

from planner.astar import astar
from planner.hybrid_astar import HybridAStarConfig, hybrid_astar
from planner.metrics import headings_from_points, path_length
from planner.parking_lot import pre_park_poses
from planner.smoothing import smooth_path
from planner.vehicle import CollisionChecker, VehicleParams

METHODS = ("astar", "astar_smoothed", "hybrid_astar")


def plan_route_with_stats(grid_map, start_pose, goal_pose, method="hybrid_astar",
                          vehicle=None, config=None, checker=None):
    """
    Plan from start_pose to goal_pose, both (x, y, heading) in world metres.

    Returns (path, stats). path is a list of (x, y, heading), or [] if no
    route was found. stats always contains 'time_s', 'nodes_expanded' and
    'directions'.

    Pass a pre-built `checker` when planning many times on the same map, so
    obstacle inflation is not redone each call.
    """
    if method not in METHODS:
        raise ValueError(f"method must be one of {METHODS}, got {method!r}")
    vehicle = vehicle or VehicleParams()
    checker = checker or CollisionChecker(grid_map, vehicle)

    if method == "hybrid_astar":
        return hybrid_astar(grid_map, start_pose, goal_pose, vehicle,
                            config or HybridAStarConfig(), checker)

    # --- Grid A* (optionally smoothed) ---
    # A* runs on the inflated grid, i.e. the car is treated as a disc with the
    # collision-circle radius centred on the rear axle.
    t0 = time.perf_counter()
    start_cell = grid_map.world_to_grid(*start_pose[:2])
    goal_cell = grid_map.world_to_grid(*goal_pose[:2])
    cells, stats = astar(checker.inflated.as_lists(), start_cell, goal_cell)

    points = [grid_map.grid_to_world(c, r) for c, r in cells]
    if method == "astar_smoothed" and points:
        points = smooth_path(points, checker)

    headings = headings_from_points(points)
    path = [(x, y, yaw) for (x, y), yaw in zip(points, headings)]
    stats["cost"] = path_length(path) if path else float("inf")  # metres, like Hybrid A*
    stats["time_s"] = time.perf_counter() - t0
    stats["directions"] = [1] * len(path)
    return path, stats


def plan_route(grid_map, start_pose, goal_pose, method="hybrid_astar", **kwargs):
    """Same as plan_route_with_stats, but returns only the path."""
    path, _ = plan_route_with_stats(grid_map, start_pose, goal_pose, method, **kwargs)
    return path


def plan_to_slot(grid_map, start_pose, slot, method="hybrid_astar",
                 vehicle=None, config=None, checker=None, offset=3.0, setup_distance_m=0.0):
    """
    Plan from start_pose to the pre-park pose of `slot` (see
    parking_lot.pre_park_poses). Both directions along the aisle are tried and
    the cheaper path is returned (path length for A*; for Hybrid A* the
    search cost, which also penalises reversing and steering).

    Note: grid A* ignores heading, so it may pick the goal that faces the
    "wrong" way along the aisle simply because it is closer.

    setup_distance_m: before reversing in, M3 drives forward until the rear
    axle is about this far past the bay centre (its turning radius). If > 0,
    a direction is skipped when the car would not fit there - e.g. driving
    towards the wall at the end of an aisle. 0 = no check.

    Returns (path, stats, goal_pose). If no route exists, path is [] and
    goal_pose is None. stats['time_s'] and stats['nodes_expanded'] are summed
    over both attempts, because both were needed to make the decision.
    """
    vehicle = vehicle or VehicleParams()
    checker = checker or CollisionChecker(grid_map, vehicle)

    best = None
    total_time, total_nodes = 0.0, 0
    for goal in pre_park_poses(slot, offset):
        if setup_distance_m > 0:
            gx, gy, gyaw = goal
            setup = (slot.x + setup_distance_m * math.cos(gyaw), gy, gyaw)
            if checker.pose_collides(*setup):
                continue  # no room to set up the reverse manoeuvre this way
        try:
            path, stats = plan_route_with_stats(grid_map, start_pose, goal, method,
                                                vehicle, config, checker)
        except ValueError:
            continue  # this candidate goal is inside an obstacle; try the other
        total_time += stats["time_s"]
        total_nodes += stats["nodes_expanded"]
        if path and (best is None or stats["cost"] < best[1]["cost"]):
            best = (path, stats, goal)

    if best is None:
        return [], {"time_s": total_time, "nodes_expanded": total_nodes, "directions": []}, None

    path, stats, goal = best
    stats = dict(stats, time_s=total_time, nodes_expanded=total_nodes)
    return path, stats, goal
