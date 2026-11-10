"""
Path smoothing for grid A* paths.

Part of M2 - Route Planner.

Idea
----
Each interior point p_i of the path is pulled by two "springs":
  - a data term that pulls it back towards its original position o_i
    (so the path does not drift away from the planned route), and
  - a smoothness term that pulls it towards the midpoint of its neighbours
    (which straightens out corners).

One update for point i (both x and y):

    p_i += w_data * (o_i - p_i) + w_smooth * (p_{i-1} + p_{i+1} - 2 * p_i)

Repeat over all points until they stop moving. The start and goal stay
fixed. If an update would push a point into an (inflated) obstacle, it is
undone, so the path stays collision-free for a disc-shaped car.

A larger w_smooth / w_data ratio gives a smoother path that cuts corners
more. Smoothing rounds corners off, but unlike Hybrid A* it gives no
guarantee that the result respects the car's minimum turning radius. The
comparison script measures exactly that.
"""


def smooth_path(points, checker, weight_data=0.05, weight_smooth=0.4,
                tolerance=1e-5, max_iterations=1000):
    """
    Smooth a list of (x, y) points.

    checker : CollisionChecker; a point is valid if checker.point_collides
              is False (the disc of the car's circle radius is free).
    Returns a new list of (x, y) tuples (same length as the input).
    """
    if len(points) < 3:
        return [tuple(p[:2]) for p in points]

    original = [list(p[:2]) for p in points]
    smoothed = [list(p[:2]) for p in points]

    for _ in range(max_iterations):
        total_change = 0.0
        for i in range(1, len(points) - 1):
            old_x, old_y = smoothed[i]
            for d in (0, 1):
                smoothed[i][d] += (
                    weight_data * (original[i][d] - smoothed[i][d])
                    + weight_smooth * (smoothed[i - 1][d] + smoothed[i + 1][d] - 2 * smoothed[i][d])
                )
            if checker.point_collides(smoothed[i][0], smoothed[i][1]):
                smoothed[i] = [old_x, old_y]  # keep the last safe position
            total_change += abs(smoothed[i][0] - old_x) + abs(smoothed[i][1] - old_y)
        if total_change < tolerance:
            break

    return [tuple(p) for p in smoothed]
