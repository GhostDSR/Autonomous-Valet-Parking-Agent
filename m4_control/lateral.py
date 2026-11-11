"""Lateral (steering) controllers. Both support reverse driving.

Conventions: pose (x, y, yaw) is the REAR AXLE centre, yaw is the vehicle
heading (nose direction), steering angle is positive to the LEFT (CCW).

Reverse trick: when reversing, the rear axle behaves like the rear axle of a
"virtual car" driving forward with heading yaw+pi and steering angle -delta.
So we run the forward law in that frame and flip the sign at the end.
"""
import numpy as np
from .path import wrap


class PurePursuit:
    name = "Pure Pursuit"

    def __init__(self, L=2.7, k_ld=0.6, ld_min=1.2, ld_max=4.0):
        self.L, self.k_ld, self.ld_min, self.ld_max = L, k_ld, ld_min, ld_max

    def ref_point(self, x, y, yaw, direction):
        return x, y                                   # rear axle

    def steer(self, seg, i, x, y, yaw, v):
        d = seg.direction
        ld = np.clip(self.k_ld * abs(v) + self.ld_min, self.ld_min, self.ld_max)
        j = min(int(np.searchsorted(seg.s, seg.s[i] + ld)), len(seg.s) - 1)
        tx, ty = seg.x[j], seg.y[j]
        th = yaw if d > 0 else yaw + np.pi
        alpha = wrap(np.arctan2(ty - y, tx - x) - th)
        ld_act = max(np.hypot(tx - x, ty - y), 1e-3)
        delta_v = np.arctan2(2 * self.L * np.sin(alpha), ld_act)
        return delta_v if d > 0 else -delta_v


class Stanley:
    name = "Stanley"

    def __init__(self, L=2.7, k=1.5, k_soft=0.5, use_ff=False):
        self.L, self.k, self.k_soft, self.use_ff = L, k, k_soft, use_ff
        if use_ff:
            self.name = "Stanley+FF"

    def ref_point(self, x, y, yaw, direction):
        if direction > 0:                             # front axle when going forward
            return x + self.L * np.cos(yaw), y + self.L * np.sin(yaw)
        return x, y                                   # rear axle when reversing

    def steer(self, seg, i, x, y, yaw, v):
        d = seg.direction
        rx, ry = self.ref_point(x, y, yaw, d)
        th = yaw if d > 0 else yaw + np.pi
        phi = seg.phi[i]
        psi_e = wrap(phi - th)
        e = -(rx - seg.x[i]) * np.sin(phi) + (ry - seg.y[i]) * np.cos(phi)   # + = left of path
        delta_v = psi_e + np.arctan2(-self.k * e, self.k_soft + abs(v))
        # Curvature feed-forward, reverse only: with a REAR-axle reference the heading
        # error is ~0 on a steady arc, so nothing else supplies the arc's steering angle.
        # (Forward, the front-axle reference already carries it via psi_e -> FF would double-count.)
        if self.use_ff and d < 0:
            delta_v += np.arctan(self.L * seg.kappa[i])
        return delta_v if d > 0 else -delta_v
