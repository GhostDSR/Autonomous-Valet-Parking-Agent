"""
Slot choice and replanning for the route planner.

Part of M2 - Route Planner.

plan_to_slot() plans to ONE slot that someone else picked. A valet agent
also has to:

  1. pick WHICH free slot to drive to            -> choose_slot()
  2. notice when the plan is no longer valid      -> check_route()
       - the target slot was taken by another car, or
       - something now blocks the rest of the route (e.g. a pedestrian
         or a car pulling out that M1's lidar has just detected)
  3. plan again from where the car is now         -> replan()

This is the "deliberative" layer reacting to new information: the reactive
layer (M3) stops the car straight away when something is too close; this
module then decides on a new route or a new slot so the car can carry on.

All poses are (x, y, heading) of the rear-axle centre in Webots metres.
"""

import math
from dataclasses import dataclass, field

from planner.parking_lot import build_parking_lot
from planner.route_planner import plan_to_slot
from planner.vehicle import CollisionChecker, VehicleParams


@dataclass
class SlotChoice:
    """A chosen slot and the route to its pre-park pose."""
    slot_id: str
    path: list                      # [(x, y, heading), ...]
    stats: dict                     # from plan_to_slot (cost, time_s, directions, ...)
    goal: tuple                     # pre-park pose the route ends at
    tried: list = field(default_factory=list)   # slot ids that were planned to


# ---------------------------------------------------------------- map updates

def build_map(occupied, extra_obstacles=(), resolution=0.25):
    """
    Occupancy grid of the lot with the given parked cars, plus any extra
    obstacles seen by the sensors.

    extra_obstacles : iterable of (cx, cy, length, width, yaw) rectangles in
                      world metres, e.g. a pedestrian as (x, y, 0.6, 0.6, 0).
    Returns (grid_map, slots).
    """
    grid_map, slots = build_parking_lot(occupied=set(occupied), resolution=resolution)
    for cx, cy, length, width, yaw in extra_obstacles:
        grid_map.add_rectangle(cx, cy, length, width, yaw)
    return grid_map, slots


# ---------------------------------------------------------------- slot choice

def rank_slots(grid_map, start_pose, slots, free_ids, checker, exclude=(), setup_distance_m=0.0):
    """
    Order the free slots from best to worst using FAST grid A*.

    Grid A* takes a few milliseconds per slot, Hybrid A* about a second, so
    we use grid A*'s route length (which follows the aisles and goes round
    parked cars, unlike a straight-line distance) to decide which slot looks
    best, and only then run the slow, drivable Hybrid A* on it.

    Returns [(route_length_m, slot_id), ...] sorted, unreachable slots left out.
    """
    ranked = []
    for sid in free_ids:
        if sid in exclude:
            continue
        path, stats, _ = plan_to_slot(grid_map, start_pose, slots[sid], "astar",
                                      checker.vehicle, None, checker,
                                      setup_distance_m=setup_distance_m)
        if path:
            ranked.append((stats["cost"], sid))
    ranked.sort()
    return ranked


def choose_slot(grid_map, start_pose, slots, free_ids, method="hybrid_astar",
                vehicle=None, config=None, checker=None, exclude=(), setup_distance_m=0.0):
    """
    Pick a free slot and plan the route to it.

    1. rank the free slots by grid-A* route length (cheap)
    2. plan with `method` (Hybrid A* by default) to the best-ranked slot;
       if that fails (e.g. no room to turn), try the next one, and so on.

    free_ids : ids of the free slots (from M1's free-slot detector)
    exclude  : ids to skip, e.g. a slot we already failed to park in
    setup_distance_m : see plan_to_slot
    Returns a SlotChoice, or None if no free slot can be reached.
    """
    vehicle = vehicle or VehicleParams()
    checker = checker or CollisionChecker(grid_map, vehicle)

    tried = []
    for _, sid in rank_slots(grid_map, start_pose, slots, free_ids, checker, set(exclude),
                             setup_distance_m):
        tried.append(sid)
        path, stats, goal = plan_to_slot(grid_map, start_pose, slots[sid], method,
                                         vehicle, config, checker,
                                         setup_distance_m=setup_distance_m)
        if path:
            return SlotChoice(sid, path, stats, goal, tried)
    return None


# ---------------------------------------------------------------- route checks

def nearest_index(path, pose, start_index=0):
    """
    Index of the path pose closest to the car, searching forward from
    start_index only (the car does not go back along its route, and a route
    with a reverse leg can pass the same spot twice).
    """
    best_i, best_d = start_index, float("inf")
    for i in range(start_index, len(path)):
        d = math.hypot(path[i][0] - pose[0], path[i][1] - pose[1])
        if d < best_d:
            best_i, best_d = i, d
    return best_i


def first_blocked_index(path, checker, start_index=0):
    """Index of the first pose (from start_index on) where the car would hit something, or None."""
    for i in range(start_index, len(path)):
        if checker.pose_collides(*path[i]):
            return i
    return None


def check_route(choice, progress_index, occupied, checker):
    """
    Is the current plan still valid?

    choice         : the SlotChoice being driven
    progress_index : index of the path pose the car has reached
    occupied       : latest set of occupied slot ids (from M1)
    checker        : CollisionChecker on the LATEST map

    Returns None if the plan is fine, otherwise a short reason string.
    """
    if choice.slot_id in occupied:
        return f"slot {choice.slot_id} is now occupied"
    blocked = first_blocked_index(choice.path, checker, progress_index)
    if blocked is not None:
        x, y, _ = choice.path[blocked]
        return f"route blocked near ({x:.1f}, {y:.1f})"
    return None


def replan(current_pose, choice, progress_index, occupied, extra_obstacles=(),
           method="hybrid_astar", vehicle=None, config=None, resolution=0.25,
           setup_distance_m=0.0):
    """
    Re-check the plan against the latest sensor picture and plan again if needed.

    Steps:
      1. rebuild the map from the latest occupied slots and extra obstacles
      2. if the plan is still valid, keep it
      3. if only the route is blocked but the slot is still free, plan a new
         route to the SAME slot from the current pose
      4. otherwise (slot taken, or the slot is now unreachable) choose a new slot

    Returns (new_choice, reason):
      new_choice is the SlotChoice to drive (the old one if nothing changed),
                 or None if no free slot can be reached;
      reason     is None if nothing changed, otherwise why we replanned.
    """
    vehicle = vehicle or VehicleParams()
    grid_map, slots = build_map(occupied, extra_obstacles, resolution)
    checker = CollisionChecker(grid_map, vehicle)

    reason = check_route(choice, progress_index, occupied, checker)
    if reason is None:
        return choice, None

    if choice.slot_id not in occupied:
        path, stats, goal = plan_to_slot(grid_map, current_pose, slots[choice.slot_id],
                                         method, vehicle, config, checker,
                                         setup_distance_m=setup_distance_m)
        if path:
            return SlotChoice(choice.slot_id, path, stats, goal, [choice.slot_id]), reason

    free_ids = [sid for sid in slots if sid not in occupied]
    new_choice = choose_slot(grid_map, current_pose, slots, free_ids, method,
                             vehicle, config, checker, exclude={choice.slot_id},
                             setup_distance_m=setup_distance_m)
    return new_choice, reason
