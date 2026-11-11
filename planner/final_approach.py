"""
Exact final approach for Hybrid A*.

Part of M2 - Route Planner.

Why
---
Hybrid A* stops as soon as it is within 0.5 m and 10 degrees of the goal,
because its search bins are 0.5 m wide and it cannot hit an exact point.
For the parking manoeuvre that is not good enough: M3 reverses into the bay
from wherever the route ends, and the room for error across the aisle is
only about 0.3 m (see pre_park_poses in parking_lot.py).

The original Hybrid A* paper fixes this with an analytic Reeds-Shepp "shot"
to the goal. Our goals are simpler - always on a straight aisle, facing
along it - so a simpler curve is enough:

  1. Hybrid A* plans to a point `approach_length` metres BEFORE the goal,
     on the same line and with the same heading (with its usual tolerance).
  2. A cubic curve joins the end of that route to the exact goal.

The cubic, in the goal's own frame (u along the goal heading, v sideways):

    v(t) = v0 + m0 t + a t^2 + b t^3,     t = u - u0 from 0 to D = -u0

starts at the route's end (offset v0, slope m0 = tan(heading error)) and
ends at the goal with v = 0 and slope 0. Solving those four conditions:

    a = -(3 v0 + 2 m0 D) / D^2        b = (2 v0 + m0 D) / D^3

Every point is checked for curvature (must be drivable: kappa <= 1/R_min)
and collisions. If either check fails, no connection is returned and the
search simply carries on to another candidate.
"""

import math

from planner.vehicle import wrap_angle


def connect_to_goal(end_pose, goal, vehicle, checker, step=0.25):
    """
    Cubic connection from end_pose to goal, both (x, y, yaw).

    Returns the list of poses after end_pose up to and including the goal,
    or None if the curve is too tight, collides, or the goal is not ahead.
    """
    gx, gy, gyaw = goal
    c, s = math.cos(gyaw), math.sin(gyaw)
    dx, dy = end_pose[0] - gx, end_pose[1] - gy
    u0 = c * dx + s * dy                  # along the goal heading (negative = behind the goal)
    v0 = -s * dx + c * dy                 # sideways offset
    heading_error = wrap_angle(end_pose[2] - gyaw)
    if u0 > -1.0 or abs(heading_error) > math.radians(45):
        return None

    D = -u0
    m0 = math.tan(heading_error)
    a = -(3 * v0 + 2 * m0 * D) / D ** 2
    b = (2 * v0 + m0 * D) / D ** 3
    kappa_max = vehicle.max_curvature

    n = max(2, math.ceil(D / step))
    poses = []
    for i in range(1, n + 1):
        t = D * i / n
        v = v0 + m0 * t + a * t ** 2 + b * t ** 3
        dv = m0 + 2 * a * t + 3 * b * t ** 2        # slope
        d2v = 2 * a + 6 * b * t                     # second derivative
        if abs(d2v) / (1 + dv * dv) ** 1.5 > kappa_max:
            return None                             # tighter than the car can turn
        u = u0 + t
        x, y = gx + u * c - v * s, gy + u * s + v * c
        yaw = wrap_angle(gyaw + math.atan(dv))
        if checker.pose_collides(x, y, yaw):
            return None
        poses.append((x, y, yaw))
    poses[-1] = (gx, gy, wrap_angle(gyaw))          # remove rounding error at the end
    return poses


def approach_point(goal, approach_length):
    """The point `approach_length` m before the goal, on the same line and heading."""
    gx, gy, gyaw = goal
    return (gx - approach_length * math.cos(gyaw), gy - approach_length * math.sin(gyaw), gyaw)
