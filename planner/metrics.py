"""
Path quality metrics used to compare planners.

Part of M2 - Route Planner.

- length            : total distance driven (m)
- curvature         : how sharply the path bends at each point (1/m).
                      Curvature 1/R means "locally like a circle of radius R".
                      A car cannot follow curvature above 1 / R_min.
- over-limit length : metres of path where curvature > the car's maximum
                      (i.e. stretches the car physically cannot follow)
- total turning     : sum of all heading changes along the path (deg)
- footprint hits    : poses where the full car rectangle (not just a point or
                      disc) would hit an obstacle
"""

import math

from planner.vehicle import wrap_angle

# Allow 5 % numerical slack before calling a point "over the limit", so
# points sampled exactly on a full-lock arc are not counted.
CURVATURE_SLACK = 1.05


def path_length(points):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:]))


def headings_from_points(points):
    """
    Heading at each point = direction of the segment leaving it (the last
    point reuses the previous segment). Used for paths that have no heading
    of their own, such as grid A* paths.
    """
    if len(points) < 2:
        return [0.0] * len(points)
    headings = [math.atan2(b[1] - a[1], b[0] - a[0]) for a, b in zip(points, points[1:])]
    headings.append(headings[-1])
    return headings


def curvature_profile(points):
    """
    Curvature at every interior point, using the circle through three
    consecutive points (Menger curvature):

        kappa = 4 * triangle_area / (|ab| * |bc| * |ca|)

    Returns (s, kappa): distance along the path of each interior point, and
    its curvature.
    """
    s_values, kappas = [], []
    s = 0.0
    for i in range(1, len(points) - 1):
        a, b, c = points[i - 1], points[i], points[i + 1]
        s += math.hypot(b[0] - a[0], b[1] - a[1])
        ab = math.hypot(b[0] - a[0], b[1] - a[1])
        bc = math.hypot(c[0] - b[0], c[1] - b[1])
        ca = math.hypot(a[0] - c[0], a[1] - c[1])
        if ab < 1e-9 or bc < 1e-9 or ca < 1e-9:
            continue  # repeated point, or a cusp that folds straight back
        cross = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
        s_values.append(s)
        kappas.append(2.0 * abs(cross) / (ab * bc * ca))  # 2*|cross| = 4*area
    return s_values, kappas


def split_by_direction(points, directions=None):
    """
    Split a path into stretches driven in one direction (forward or reverse).

    At a forward <-> reverse switch the path folds back on itself (a "cusp").
    That is not a bend the car has to steer through, so curvature must be
    measured within each stretch, never across a cusp.
    """
    if not directions:
        return [points]
    pieces, current = [], [points[0]]
    for i in range(1, len(points)):
        if directions[i] != directions[i - 1]:
            pieces.append(current)
            current = [points[i - 1]]  # the cusp point starts the next stretch
        current.append(points[i])
    pieces.append(current)
    return pieces


def length_over_limit(points, limit):
    """
    Metres of path where curvature exceeds `limit`: each over-limit point
    counts half of each segment touching it.
    """
    total = 0.0
    for i in range(1, len(points) - 1):
        _, k = curvature_profile(points[i - 1:i + 2])
        if k and k[0] > limit:
            a, b, c = points[i - 1], points[i], points[i + 1]
            total += 0.5 * (math.hypot(b[0] - a[0], b[1] - a[1]) + math.hypot(c[0] - b[0], c[1] - b[1]))
    return total


def compute_metrics(path, vehicle, checker, goal=None, directions=None):
    """
    path       : list of (x, y, yaw) poses (rear-axle reference point).
    vehicle    : VehicleParams (for the curvature limit).
    checker    : CollisionChecker (for the full-footprint check).
    goal       : optional goal pose, to report how close the path ends.
    directions : optional +1/-1 per pose (forward/reverse).
    """
    if not path:
        return {"found": False}

    points = [(p[0], p[1]) for p in path]
    limit = vehicle.max_curvature * CURVATURE_SLACK
    kappas, over_limit = [], 0.0
    for piece in split_by_direction(points, directions):
        kappas.extend(curvature_profile(piece)[1])
        over_limit += length_over_limit(piece, limit)

    turning = sum(abs(wrap_angle(b[2] - a[2])) for a, b in zip(path, path[1:]))
    switches = 0
    if directions:
        switches = sum(1 for a, b in zip(directions, directions[1:]) if a != b)

    metrics = {
        "found": True,
        "length_m": path_length(points),
        "max_curvature": max(kappas) if kappas else 0.0,
        "over_limit_m": over_limit,
        "total_turning_deg": math.degrees(turning),
        "direction_switches": switches,
        "footprint_hits": sum(checker.pose_collides(x, y, yaw) for x, y, yaw in path),
    }
    if goal is not None:
        end = path[-1]
        metrics["goal_pos_error_m"] = math.hypot(end[0] - goal[0], end[1] - goal[1])
        metrics["goal_yaw_error_deg"] = abs(math.degrees(wrap_angle(end[2] - goal[2])))
    return metrics
