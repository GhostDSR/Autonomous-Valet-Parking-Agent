"""
Hybrid A* planner for a car-like (Ackermann) vehicle.

Part of M2 - Route Planner.

The problem with plain A*
-------------------------
Grid A* moves from cell centre to cell centre and can turn 45 or 90 degrees
on the spot. A real car cannot: it has a minimum turning radius. A path with
sharp corners is not drivable.

The Hybrid A* idea (Dolgov et al., 2010)
----------------------------------------
1. The state is continuous: (x, y, yaw) of the rear axle, not a cell.
2. New states are made by actually *driving* the car model for a short
   distance with a fixed steering angle (a "motion primitive"): forward or
   reverse, steering hard left ... straight ... hard right. Every path built
   from these pieces is drivable by construction.
3. To keep the search finite, the continuous states are binned into a coarse
   3D grid (x cell, y cell, heading bin). Only one state is kept per bin:
   once a bin is expanded ("closed") it is not expanded again.
   That binning is the "hybrid" part: continuous states, discrete bookkeeping.
4. Like A*, nodes are expanded in order of f = g + h:
     g = cost so far (distance, with extra cost for reversing, steering and
         changing direction, so the planner prefers calm, forward driving)
     h = distance-to-goal on the 2D grid around obstacles (a Dijkstra search
         run once from the goal), which guides the search along the aisles.
5. The search stops when a state lands within a small position and heading
   tolerance of the goal. Optionally (config.final_approach_m > 0) it
   instead searches to a point just before the goal and joins it to the
   exact goal with a cubic curve (final_approach.py).

Simplifications compared to the paper (keep in mind for the viva):
- No analytic Reeds-Shepp "shot" to the goal; a goal tolerance is used
  instead, or the simpler cubic final approach.
- Only the obstacle-aware 2D heuristic is used, not the non-holonomic
  (Reeds-Shepp) heuristic.
- No post-optimisation (gradient smoothing) of the final path.
"""

import heapq
import itertools
import math
import time
from dataclasses import dataclass

from planner.final_approach import approach_point, connect_to_goal
from planner.vehicle import CollisionChecker, VehicleParams, move, wrap_angle


@dataclass
class HybridAStarConfig:
    # Search discretisation
    xy_resolution: float = 0.5          # m, size of the (x, y) bins
    yaw_resolution_deg: float = 5.0     # deg, size of the heading bins
    step_length: float = 1.0            # m driven by one motion primitive (> bin diagonal)
    sub_step: float = 0.25              # m between collision checks along a primitive
    n_steer: int = 5                    # steering angles from -max to +max (odd -> includes 0)
    allow_reverse: bool = True

    # Cost shaping (all relative to 1 metre of forward driving)
    reverse_cost: float = 2.0           # reversing costs this many times more per metre
    steer_cost: float = 0.2             # per metre, at full steering lock
    steer_change_cost: float = 0.5      # for swinging the wheel from one lock to centre
    direction_switch_cost: float = 5.0  # for each forward <-> reverse switch

    # Heuristic
    heuristic_weight: float = 1.0       # > 1 searches faster but may give longer paths
    heuristic_inflation: float = 0.5    # m, obstacle growth for the 2D heuristic grid

    # Goal and limits
    goal_xy_tolerance: float = 0.5      # m
    goal_yaw_tolerance_deg: float = 10.0
    max_expansions: int = 100_000

    # Exact final approach (see final_approach.py). 0 = off: stop within the
    # tolerance above. > 0: search to a point this many metres before the goal,
    # then join it to the exact goal with a drivable cubic curve.
    final_approach_m: float = 0.0


class _Node:
    """One search state. `trajectory` holds the poses driven to reach it from its parent."""

    __slots__ = ("x", "y", "yaw", "direction", "steer", "g", "parent", "trajectory")

    def __init__(self, x, y, yaw, direction, steer, g, parent, trajectory):
        self.x, self.y, self.yaw = x, y, yaw
        self.direction = direction  # +1 forward, -1 reverse, 0 = start (no move yet)
        self.steer = steer
        self.g = g
        self.parent = parent
        self.trajectory = trajectory


# ---------------------------------------------------------------------------
# Heuristic: 2D shortest distance around obstacles (Dijkstra from the goal)
# ---------------------------------------------------------------------------

def holonomic_heuristic(grid_map, goal_xy, inflation):
    """
    Distance (m) from every cell to the goal, moving 8-connected on the grid
    and avoiding obstacles. Computed once per goal with Dijkstra's algorithm
    (= A* without a heuristic, run until every reachable cell is done).

    Obstacles are inflated a little, so the heuristic does not "see" gaps
    between parked cars that the car could never fit through.
    """
    occ = grid_map.inflate(inflation).as_lists()
    rows, cols, res = grid_map.rows, grid_map.cols, grid_map.resolution
    dist = [[math.inf] * cols for _ in range(rows)]

    gc, gr = grid_map.world_to_grid(*goal_xy)
    if not grid_map.in_bounds(gc, gr) or occ[gr][gc]:
        raise ValueError("goal is inside an obstacle on the heuristic grid")

    moves = [(dc, dr, res * math.hypot(dc, dr))
             for dc in (-1, 0, 1) for dr in (-1, 0, 1) if (dc, dr) != (0, 0)]
    dist[gr][gc] = 0.0
    heap = [(0.0, gc, gr)]
    while heap:
        d, c, r = heapq.heappop(heap)
        if d > dist[r][c]:
            continue  # stale heap entry
        for dc, dr, cost in moves:
            nc, nr = c + dc, r + dr
            if 0 <= nc < cols and 0 <= nr < rows and not occ[nr][nc]:
                nd = d + cost
                if nd < dist[nr][nc]:
                    dist[nr][nc] = nd
                    heapq.heappush(heap, (nd, nc, nr))
    return dist


# ---------------------------------------------------------------------------
# Hybrid A* search
# ---------------------------------------------------------------------------

def _motion_primitives(vehicle, config):
    """All (direction, steering angle) pairs the planner may use."""
    if config.n_steer == 1:
        steers = [0.0]
    else:
        steers = [-vehicle.max_steer + 2 * vehicle.max_steer * i / (config.n_steer - 1)
                  for i in range(config.n_steer)]
    directions = (1, -1) if config.allow_reverse else (1,)
    return [(d, s) for d in directions for s in steers]


def _drive(node, direction, steer, vehicle, config, checker):
    """
    Drive one motion primitive from `node`. Returns the list of poses along
    it, or None if any of them collides.
    """
    n = max(1, int(math.ceil(config.step_length / config.sub_step)))
    ds = direction * config.step_length / n
    x, y, yaw = node.x, node.y, node.yaw
    poses = []
    for _ in range(n):
        x, y, yaw = move(x, y, yaw, ds, steer, vehicle.wheelbase)
        if checker.pose_collides(x, y, yaw):
            return None
        poses.append((x, y, yaw))
    return poses


def _step_cost(parent, direction, steer, vehicle, config):
    """Cost g added by driving one primitive from `parent`."""
    length = config.step_length
    cost = length * (config.reverse_cost if direction < 0 else 1.0)
    cost += config.steer_cost * length * abs(steer) / vehicle.max_steer
    cost += config.steer_change_cost * abs(steer - parent.steer) / vehicle.max_steer
    if parent.direction != 0 and direction != parent.direction:
        cost += config.direction_switch_cost
    return cost


def _reconstruct(goal_node):
    """Follow parent links back to the start and return (poses, directions)."""
    chain = []
    node = goal_node
    while node is not None:
        chain.append(node)
        node = node.parent
    chain.reverse()

    start = chain[0]
    poses = [(start.x, start.y, start.yaw)]
    directions = [chain[1].direction if len(chain) > 1 else 1]
    for node in chain[1:]:
        poses.extend(node.trajectory)
        directions.extend([node.direction] * len(node.trajectory))
    return poses, directions


def hybrid_astar(grid_map, start, goal, vehicle=None, config=None, checker=None):
    """
    Plan a drivable path from `start` to `goal`.

    grid_map : GridMap of the lot (raw obstacles, NOT inflated).
    start, goal : (x, y, yaw) rear-axle poses in world metres / radians.
    vehicle  : VehicleParams (defaults used if None).
    config   : HybridAStarConfig (defaults used if None).
    checker  : optional pre-built CollisionChecker (saves rebuilding it).

    Returns (path, stats):
      path  : list of (x, y, yaw) poses, spaced `config.sub_step` apart, or []
      stats : dict with
              'cost', 'nodes_expanded', 'time_s', 'reached_goal',
              'directions' : +1 / -1 for each pose (forward / reverse)
    """
    t0 = time.perf_counter()
    vehicle = vehicle or VehicleParams()
    config = config or HybridAStarConfig()
    checker = checker or CollisionChecker(grid_map, vehicle)
    stats = {"cost": math.inf, "nodes_expanded": 0, "time_s": 0.0,
             "reached_goal": False, "directions": []}

    for name, pose in (("start", start), ("goal", goal)):
        if checker.pose_collides(*pose):
            raise ValueError(f"{name} pose {pose} collides with an obstacle")

    # With a final approach, the search aims for a point just before the goal.
    final_goal = goal
    if config.final_approach_m > 0:
        goal = approach_point(final_goal, config.final_approach_m)
        if checker.pose_collides(*goal):
            raise ValueError(f"approach pose {goal} collides with an obstacle")

    h_grid = holonomic_heuristic(grid_map, goal[:2], config.heuristic_inflation)

    def h(x, y):
        col, row = grid_map.world_to_grid(x, y)
        if not grid_map.in_bounds(col, row):
            return math.inf
        return h_grid[row][col]

    yaw_res = math.radians(config.yaw_resolution_deg)
    n_yaw_bins = int(round(2 * math.pi / yaw_res))
    xy_res = config.xy_resolution

    def bin_key(x, y, yaw):
        return (int(math.floor(x / xy_res)),
                int(math.floor(y / xy_res)),
                int(round(yaw / yaw_res)) % n_yaw_bins)

    goal_yaw_tol = math.radians(config.goal_yaw_tolerance_deg)

    def at_goal(node):
        return (math.hypot(node.x - goal[0], node.y - goal[1]) <= config.goal_xy_tolerance
                and abs(wrap_angle(node.yaw - goal[2])) <= goal_yaw_tol)

    primitives = _motion_primitives(vehicle, config)
    w = config.heuristic_weight

    start_node = _Node(start[0], start[1], wrap_angle(start[2]), 0, 0.0, 0.0, None, [])
    h_start = h(start[0], start[1])
    if h_start == math.inf:
        stats["time_s"] = time.perf_counter() - t0
        return [], stats  # goal not reachable even ignoring the car's shape

    tie_breaker = itertools.count()  # avoids comparing _Node objects in the heap
    open_heap = [(w * h_start, next(tie_breaker), start_node)]
    best_g = {bin_key(*start[:2], start_node.yaw): 0.0}
    closed = set()

    while open_heap:
        _, _, node = heapq.heappop(open_heap)
        key = bin_key(node.x, node.y, node.yaw)
        if key in closed:
            continue
        closed.add(key)
        stats["nodes_expanded"] += 1

        if at_goal(node):
            path, directions = _reconstruct(node)
            cost = node.g
            tail = []
            if config.final_approach_m > 0:
                # Join to the exact goal; if that is not possible from this
                # node, just keep searching (this node is expanded as normal).
                tail = connect_to_goal(path[-1], final_goal, vehicle, checker, config.sub_step)
                cost += config.final_approach_m
            if tail is not None:
                stats.update(cost=cost, reached_goal=True, directions=directions + [1] * len(tail),
                             time_s=time.perf_counter() - t0)
                return path + tail, stats

        if stats["nodes_expanded"] >= config.max_expansions:
            break

        for direction, steer in primitives:
            poses = _drive(node, direction, steer, vehicle, config, checker)
            if poses is None:
                continue
            x, y, yaw = poses[-1]
            child_key = bin_key(x, y, yaw)
            if child_key in closed:
                continue
            h_child = h(x, y)
            if h_child == math.inf:
                continue
            g = node.g + _step_cost(node, direction, steer, vehicle, config)
            if g < best_g.get(child_key, math.inf):
                best_g[child_key] = g
                child = _Node(x, y, yaw, direction, steer, g, node, poses)
                heapq.heappush(open_heap, (g + w * h_child, next(tie_breaker), child))

    stats["time_s"] = time.perf_counter() - t0
    return [], stats
