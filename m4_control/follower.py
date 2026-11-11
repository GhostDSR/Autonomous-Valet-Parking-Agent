"""PathFollower: ties lateral controller + speed profile + speed PID together.

Per control step:   step(x, y, yaw, v, dt)  ->  Command
  steer  : steering angle [rad], + = left
  accel  : acceleration along the CURRENT travel direction [m/s^2]
           (+ = throttle, - = brake) -- already the PID output
  v_ref  : signed speed reference (negative while reversing)
"""
from dataclasses import dataclass
import numpy as np
from .path import wrap
from .pid import PID


@dataclass
class FollowerConfig:
    max_steer: float = 0.55        # rad
    v_fwd: float = 2.5             # m/s   cruise speed, forward
    v_rev: float = 1.0             # m/s   cruise speed, reverse
    v_creep: float = 0.3           # m/s   minimum speed while still approaching a stop
    a_lat: float = 1.0             # m/s^2 comfortable lateral accel (curve speed limit)
    a_dec: float = 1.0             # m/s^2 planned deceleration for stopping
    a_ramp_up: float = 1.0         # m/s^2 speed-reference ramp up
    a_ramp_dn: float = 2.0         # m/s^2 speed-reference ramp down
    stop_margin: float = 0.05      # m
    stop_tol: float = 0.06         # m     'arrived' distance
    v_stop: float = 0.05           # m/s   'stopped' speed
    dwell: float = 0.8             # s     pause at a gear change (pre-steers meanwhile)
    kp: float = 1.5
    ki: float = 0.8
    kd: float = 0.0
    ff_gain: float = 1.0           # acceleration feed-forward from the speed profile
    a_acc: float = 2.0             # m/s^2 PID output limit (throttle)
    a_brk: float = 4.0             # m/s^2 PID output limit (brake)


@dataclass
class Command:
    steer: float
    accel: float
    v_ref: float
    direction: int
    state: str
    cte: float
    heading_err: float
    seg: int
    remaining: float
    done: bool
    brake_hold: bool


class PathFollower:
    def __init__(self, path, lateral, cfg=None):
        self.path, self.lat = path, lateral
        self.cfg = cfg or FollowerConfig()
        c = self.cfg
        self.pid = PID(c.kp, c.ki, c.kd, -c.a_brk, c.a_acc)
        self.k, self.i, self.i_prog = 0, 0, 0
        self.vref = 0.0            # magnitude of the ramped speed reference
        self.state = "DRIVE"       # DRIVE | DWELL | DONE
        self.t_dwell = 0.0

    @property
    def done(self):
        return self.state == "DONE"

    def _speed_target(self, seg, i, rem):
        c = self.cfg
        vmax = c.v_fwd if seg.direction > 0 else c.v_rev
        j = min(int(np.searchsorted(seg.s, seg.s[i] + 2.0)), len(seg.s) - 1)
        kap = max(np.max(np.abs(seg.kappa[i:j + 1])), 1e-6)
        v_curve = np.sqrt(c.a_lat / kap)
        v_stop = np.sqrt(2 * c.a_dec * max(rem - c.stop_margin, 0.0))
        t = min(vmax, v_curve, v_stop)
        return max(t, c.v_creep) if rem > c.stop_tol else 0.0

    def step(self, x, y, yaw, v, dt, estop=False):
        c = self.cfg
        seg = self.path.segments[self.k]
        d = seg.direction

        # steering index: nearest point to the controller's own reference point
        # (rear axle for Pure Pursuit, front axle for forward Stanley)
        rx, ry = self.lat.ref_point(x, y, yaw, d)
        self.i = seg.nearest(rx, ry, self.i)
        i = self.i
        phi = seg.phi[i]
        cte = -(rx - seg.x[i]) * np.sin(phi) + (ry - seg.y[i]) * np.cos(phi)
        heading_err = wrap(phi - (yaw if d > 0 else yaw + np.pi))
        # progress index: ALWAYS the rear axle (the path/cusp/slot are rear-axle poses)
        self.i_prog = seg.nearest(x, y, self.i_prog)
        ip = self.i_prog
        rem = max(seg.s_end - seg.s[ip], 0.0)
        steer = float(np.clip(self.lat.steer(seg, i, x, y, yaw, v), -c.max_steer, c.max_steer))

        # ---- state machine -------------------------------------------------
        target = 0.0
        if self.state == "DRIVE" and not estop:
            if rem <= c.stop_tol and abs(v) < c.v_stop:          # segment finished
                if self.k + 1 < len(self.path.segments):
                    self.k, self.i, self.i_prog = self.k + 1, 0, 0
                    self.state, self.t_dwell = "DWELL", 0.0
                    self.vref = 0.0
                    self.pid.reset()
                else:
                    self.state = "DONE"
            else:
                target = self._speed_target(seg, ip, rem)
        elif self.state == "DWELL" and not estop:
            self.t_dwell += dt
            if self.t_dwell >= c.dwell:
                self.state = "DRIVE"

        # ---- speed reference ramp + PID -----------------------------------
        vref_prev = self.vref
        if target > self.vref:
            self.vref = min(target, self.vref + c.a_ramp_up * dt)
        else:
            self.vref = max(target, self.vref - c.a_ramp_dn * dt)
        a_ff = (self.vref - vref_prev) / dt          # acceleration feed-forward
        if estop:
            self.vref = 0.0
            self.pid.reset()
            accel = -c.a_brk
        else:
            err = self.vref - d * v        # error in the travel-direction frame
            accel = float(np.clip(self.pid.step(err, dt) + c.ff_gain * a_ff, -c.a_brk, c.a_acc))
        hold = (self.vref == 0.0) and abs(v) < 0.1
        return Command(steer, accel, d * self.vref, d, self.state, cte, heading_err,
                       self.k, rem, self.done, hold)
