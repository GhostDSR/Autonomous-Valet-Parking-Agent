"""Kinematic-bicycle plant with actuator limits + sensor noise, for offline experiments.

Realism that matters for controller comparison:
  * steering: saturation, rate limit and first-order lag
  * drivetrain: first-order acceleration lag, drag, brake limit
  * sensors: Gaussian GPS / compass / speed noise
"""
from dataclasses import dataclass
import numpy as np
from .follower import PathFollower, FollowerConfig
from .path import wrap
from .estimator import PoseFilter


@dataclass
class VehicleParams:
    L: float = 2.7
    max_steer: float = 0.55
    steer_rate: float = 1.0       # rad/s
    steer_tau: float = 0.12       # s
    a_acc: float = 2.0
    a_brk: float = 4.0
    accel_tau: float = 0.25       # s
    drag: float = 0.15            # 1/s


class Bicycle:
    def __init__(self, x, y, yaw, p: VehicleParams):
        self.x, self.y, self.yaw, self.p = x, y, yaw, p
        self.v = 0.0
        self.delta = 0.0
        self.a = 0.0

    def step(self, steer_cmd, accel_dir, direction, dt):
        p = self.p
        tgt = np.clip(steer_cmd, -p.max_steer, p.max_steer)
        rate = np.clip((tgt - self.delta) / p.steer_tau, -p.steer_rate, p.steer_rate)
        self.delta += rate * dt

        if accel_dir >= 0:
            a_t = direction * min(accel_dir, p.a_acc)
        else:
            a_t = -np.sign(self.v) * min(-accel_dir, p.a_brk) if abs(self.v) > 1e-3 else 0.0
        self.a += (a_t - self.a) * dt / p.accel_tau
        v_new = self.v + (self.a - p.drag * self.v) * dt
        if accel_dir < 0 and self.v * v_new <= 0:        # brakes can't reverse the car
            v_new, self.a = 0.0, 0.0
        self.v = v_new

        self.x += self.v * np.cos(self.yaw) * dt
        self.y += self.v * np.sin(self.yaw) * dt
        self.yaw = wrap(self.yaw + self.v * np.tan(self.delta) / p.L * dt)


def run_episode(path, lateral, cfg=None, vp=None, gps_sigma=0.0, yaw_sigma=0.0,
                speed_sigma=0.0, init_offset=(0.3, 0.05), dt=0.02, t_max=150.0, seed=0,
                use_filter=False):
    """Simulate one parking run. init_offset = (lateral [m], heading [rad]) start error."""
    rng = np.random.default_rng(seed)
    vp = vp or VehicleParams()
    cfg = cfg or FollowerConfig(max_steer=vp.max_steer)
    fol = PathFollower(path, lateral, cfg)
    filt = PoseFilter(vp.L) if use_filter else None
    last_steer = 0.0

    s0 = path.segments[0]
    yaw0 = s0.phi[0] + (np.pi if s0.direction < 0 else 0.0)
    nx, ny = -np.sin(s0.phi[0]), np.cos(s0.phi[0])
    car = Bicycle(s0.x[0] + init_offset[0] * nx, s0.y[0] + init_offset[0] * ny,
                  wrap(yaw0 + init_offset[1]), vp)

    keys = ["t", "x", "y", "yaw", "v", "delta", "v_ref", "accel", "cte", "head_err",
            "seg", "dir", "state"]
    log = {k: [] for k in keys}
    t = 0.0
    while t < t_max and not fol.done:
        xm = car.x + rng.normal(0, gps_sigma)
        ym = car.y + rng.normal(0, gps_sigma)
        yawm = car.yaw + rng.normal(0, yaw_sigma)
        vm = car.v + rng.normal(0, speed_sigma)
        if filt:
            xm, ym, yawm = filt.update(xm, ym, yawm, vm, last_steer, dt)
        cmd = fol.step(xm, ym, yawm, vm, dt)
        last_steer = cmd.steer
        car.step(cmd.steer, cmd.accel, cmd.direction, dt)

        # evaluation uses TRUE pose and the rear-axle point for every controller
        seg = path.segments[cmd.seg]
        cte, k = seg.lateral_error(car.x, car.y)
        th = car.yaw if seg.direction > 0 else car.yaw + np.pi
        for key, val in zip(keys, [t, car.x, car.y, car.yaw, car.v, car.delta, cmd.v_ref,
                                   cmd.accel, cte, wrap(seg.phi[k] - th), cmd.seg,
                                   cmd.direction, cmd.state]):
            log[key].append(val)
        t += dt
    out = {k: np.array(v) for k, v in log.items()}
    out["done"] = fol.done
    out["dt"] = dt
    return out
