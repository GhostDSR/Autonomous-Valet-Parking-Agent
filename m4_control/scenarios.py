"""Synthetic M2-style paths (forward drive + reverse-in manoeuvre) for testing M4."""
import numpy as np
from .path import Path, fillet_polyline, resample


def reverse_in_arc(cusp_xy, cusp_yaw, R, turn, depth, ds=0.05):
    """Quarter-circle reverse arc + straight into the slot, starting at the cusp pose."""
    psi = cusp_yaw + np.pi                       # travel direction while reversing
    s = 1 if turn == "left" else -1
    n = np.array([-np.sin(psi), np.cos(psi)])
    P = np.asarray(cusp_xy, float)
    C = P + s * R * n
    phi0 = psi - s * np.pi / 2
    t = np.linspace(0, np.pi / 2, int(R * np.pi / 2 / ds) + 2)
    arc = C + R * np.c_[np.cos(phi0 + s * t), np.sin(phi0 + s * t)]
    psi_end = psi + s * np.pi / 2
    straight = arc[-1] + np.outer(np.arange(1, int(depth / ds) + 1) * ds,
                                  [np.cos(psi_end), np.sin(psi_end)])
    return np.vstack([arc, straight])


def _build(fwd_pts, cusp_yaw, R, turn, depth, fillet_r=6.0):
    fwd = resample(fillet_polyline(fwd_pts, fillet_r), 0.05)
    rev = reverse_in_arc(fwd[-1], cusp_yaw, R, turn, depth)
    xy = np.vstack([fwd, rev])
    dirn = np.r_[np.ones(len(fwd)), -np.ones(len(rev))]
    return xy, dirn


def scenario_straight():
    """Drive down the aisle, reverse-in with a left-hand arc."""
    xy, d = _build([[0, 0], [28, 0]], 0.0, R=5.5, turn="left", depth=4.0)
    return Path.from_points(xy, d)


def scenario_L_route():
    """Entrance -> two corners (A*-style polyline, filleted) -> reverse-in right-hand arc."""
    xy, d = _build([[0, 0], [20, 0], [20, 12], [6, 12]], np.pi, R=5.5, turn="right", depth=4.0)
    return Path.from_points(xy, d)


def scenario_tight():
    """Same as straight but R close to the car's minimum turning radius (4.4 m)."""
    xy, d = _build([[0, 0], [28, 0]], 0.0, R=4.8, turn="left", depth=4.0)
    return Path.from_points(xy, d)


SCENARIOS = {"straight": scenario_straight, "L_route": scenario_L_route, "tight": scenario_tight}
