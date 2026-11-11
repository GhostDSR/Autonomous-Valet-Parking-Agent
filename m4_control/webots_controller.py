"""M4 Webots controller  --  path following with Pure Pursuit / Stanley + speed PID.

SETUP
  1. Copy the whole  m4_control/  folder into  <your_project>/controllers/
  2. Create  controllers/m4_parking/m4_parking.py  containing only:
         from m4_control.webots_controller import main
         main()
     and set the car's  controller  field to "m4_parking".
  3. Export M2's path as CSV  (header: x,y,dir ; dir=+1 forward, -1 reverse) and
     point CONFIG["path_csv"] at it. Coordinates must be the world ground frame (ENU:
     x = east, y = north -- default for Webots >= R2022a).
  4. FIRST RUN with CONFIG["mode"] = "calibrate" and read the printed hints
     (compass yaw offset, steering sign), copy them into CONFIG, then switch to "run".

Everything below the CONFIG block is device plumbing; the control logic lives in
follower.py / lateral.py and is identical to what was tested offline.
"""
import csv
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import numpy as np
from vehicle import Driver                      # Webots automobile library

from m4_control.estimator import PoseFilter
from m4_control.follower import FollowerConfig, PathFollower
from m4_control.lateral import PurePursuit, Stanley
from m4_control.path import Path, wrap

CONFIG = dict(
    mode="run",                    # "run" | "calibrate"
    path_csv="path_m2.csv",
    controller="stanley_ff",       # "pure_pursuit" | "stanley" | "stanley_ff"
    log_csv="webots_run.csv",
    # --- vehicle geometry (measure from your PROTO / world) ---------------------
    wheelbase=2.9,                 # m
    max_steer=0.5,                 # rad
    gps_forward_of_rear_axle=0.0,  # m: how far ahead of the rear axle the GPS node sits
    # --- device names / conventions --------------------------------------------
    gps_name="gps",
    compass_name="compass",
    steer_sign=-1.0,               # Webots: +angle = right turn -> we use +left, so -1. Verify in calibrate mode.
    compass_yaw_offset=0.0,        # rad, from calibrate mode
    # --- actuation ---------------------------------------------------------------
    speed_mode="pedals",           # "pedals": our PID drives throttle/brake/gear
                                   # "cruise": send v_ref to Webots' built-in cruise control (our PID unused)
    # --- safety ------------------------------------------------------------------
    estop_sensors=[],              # e.g. ["ds_rear", "ds_front"]
    estop_dist=0.5,                # m  (sensor lookup table should output metres)
    # --- optional ----------------------------------------------------------------
    use_pose_filter=False,
)


# ============================================================== device adapters
class Sensors:
    def __init__(self, drv, cfg, dt_ms):
        self.cfg = cfg
        self.gps = drv.getDevice(cfg["gps_name"]); self.gps.enable(dt_ms)
        self.compass = drv.getDevice(cfg["compass_name"]); self.compass.enable(dt_ms)
        self.ds = []
        for n in cfg["estop_sensors"]:
            s = drv.getDevice(n); s.enable(dt_ms); self.ds.append(s)

    def yaw(self):
        c = self.compass.getValues()
        # compass returns the world 'north' vector in the robot frame; ENU world, x-forward robot
        return wrap(math.atan2(c[0], c[1]) + self.cfg["compass_yaw_offset"])

    def pose(self):
        """Rear-axle (x, y), yaw, signed speed along the heading."""
        g = self.gps.getValues()
        yaw = self.yaw()
        off = self.cfg["gps_forward_of_rear_axle"]
        x, y = g[0] - off * math.cos(yaw), g[1] - off * math.sin(yaw)
        sv = self.gps.getSpeedVector()
        v = sv[0] * math.cos(yaw) + sv[1] * math.sin(yaw)       # signed
        return x, y, yaw, v

    def obstacle_too_close(self):
        return any(0.0 < s.getValue() < self.cfg["estop_dist"] for s in self.ds)


class DriverCar:
    """Webots `Driver` (BMW X5 etc.): steering angle + throttle / brake / gear."""

    def __init__(self, drv, cfg, fcfg):
        self.d, self.cfg, self.f = drv, cfg, fcfg
        self.gear = 1
        self.d.setSteeringAngle(0.0)

    def apply(self, cmd, v_meas):
        c = self.cfg
        self.d.setSteeringAngle(c["steer_sign"] * cmd.steer)
        if c["speed_mode"] == "cruise":
            self.d.setCruisingSpeed(cmd.v_ref * 3.6)           # km/h, signed
            return
        if cmd.brake_hold:
            self.d.setThrottle(0.0); self.d.setBrakeIntensity(1.0); return
        if abs(v_meas) < 0.15 and cmd.direction != self.gear:   # only change gear when stopped
            self.gear = cmd.direction
            self.d.setGear(-1 if self.gear < 0 else 1)
        if cmd.accel >= 0:
            self.d.setThrottle(min(cmd.accel / self.f.a_acc, 1.0)); self.d.setBrakeIntensity(0.0)
        else:
            self.d.setThrottle(0.0); self.d.setBrakeIntensity(min(-cmd.accel / self.f.a_brk, 1.0))

    def steering_angle(self):
        return self.cfg["steer_sign"] * self.d.getSteeringAngle()


def ackermann_angles(delta, L, T):
    """For a CUSTOM car with two steered front wheels: (left, right) wheel angles for a
    virtual bicycle steering angle delta (+ = left). T = track width."""
    if abs(delta) < 1e-6:
        return 0.0, 0.0
    R = L / math.tan(delta)                 # signed turn radius of the rear-axle centre
    return math.atan(L / (R - T / 2)), math.atan(L / (R + T / 2))


# ============================================================== main loop
def make_lateral(cfg):
    L = cfg["wheelbase"]
    return {"pure_pursuit": lambda: PurePursuit(L=L),
            "stanley": lambda: Stanley(L=L),
            "stanley_ff": lambda: Stanley(L=L, use_ff=True)}[cfg["controller"]]()


def calibrate(drv, sensors, car, dt):
    """Drive straight, then arc left. Prints the numbers to put into CONFIG."""
    print("[calibrate] driving straight for 3 s ...")
    x0, y0, _, _ = sensors.pose()
    yaw_c = []
    for _ in range(int(3.0 / dt)):
        if drv.step() == -1: return
        drv.setSteeringAngle(0.0); drv.setCruisingSpeed(5.0)
        yaw_c.append(sensors.yaw())
    x1, y1, _, _ = sensors.pose()
    course = math.atan2(y1 - y0, x1 - x0)
    comp = math.atan2(np.mean(np.sin(yaw_c)), np.mean(np.cos(yaw_c)))
    print(f"[calibrate]  GPS course = {course:+.3f} rad, compass yaw = {comp:+.3f} rad")
    print(f"[calibrate]  -> set compass_yaw_offset = {wrap(course - comp):+.3f}")

    print("[calibrate] steering 'left' (+0.3 rad in OUR convention) for 3 s ...")
    yaw_a = sensors.yaw() + CONFIG["compass_yaw_offset"] * 0
    for _ in range(int(3.0 / dt)):
        if drv.step() == -1: return
        drv.setSteeringAngle(CONFIG["steer_sign"] * 0.3); drv.setCruisingSpeed(5.0)
    d_yaw = wrap(sensors.yaw() - yaw_a)
    print(f"[calibrate]  yaw changed by {d_yaw:+.3f} rad -> "
          f"{'steer_sign is CORRECT' if d_yaw > 0 else 'FLIP steer_sign (multiply by -1)'}")
    drv.setCruisingSpeed(0.0); drv.setBrakeIntensity(1.0)


def main():
    cfg = CONFIG
    drv = Driver()
    dt_ms = int(drv.getBasicTimeStep())
    dt = dt_ms / 1000.0
    sensors = Sensors(drv, cfg, dt_ms)
    fcfg = FollowerConfig(max_steer=cfg["max_steer"])
    car = DriverCar(drv, cfg, fcfg)

    if cfg["mode"] == "calibrate":
        calibrate(drv, sensors, car, dt)
        return

    path = Path.from_csv(cfg["path_csv"])
    follower = PathFollower(path, make_lateral(cfg), fcfg)
    filt = PoseFilter(cfg["wheelbase"]) if cfg["use_pose_filter"] else None
    last_steer = 0.0

    cols = ["t", "x", "y", "yaw", "v", "delta", "v_ref", "accel", "cte", "head_err", "seg", "dir", "state"]
    fh = open(cfg["log_csv"], "w", newline="")
    wr = csv.writer(fh); wr.writerow(cols)

    t = 0.0
    while drv.step() != -1:
        x, y, yaw, v = sensors.pose()
        if filt:
            x, y, yaw = filt.update(x, y, yaw, v, last_steer, dt)
        estop = sensors.obstacle_too_close()                 # M3 can also set this
        cmd = follower.step(x, y, yaw, v, dt, estop=estop)
        last_steer = cmd.steer
        car.apply(cmd, v)
        wr.writerow([f"{t:.3f}", f"{x:.4f}", f"{y:.4f}", f"{yaw:.4f}", f"{v:.4f}",
                     f"{car.steering_angle():.4f}", f"{cmd.v_ref:.4f}", f"{cmd.accel:.4f}",
                     f"{cmd.cte:.4f}", f"{cmd.heading_err:.4f}", cmd.seg, cmd.direction, cmd.state])
        t += dt
        if cmd.done:
            print(f"[M4] parked. rear-axle pose = ({x:.2f}, {y:.2f}, {math.degrees(yaw):.1f} deg)")
            fh.flush()
            # keep brakes on, stop logging
            while drv.step() != -1:
                drv.setThrottle(0.0); drv.setBrakeIntensity(1.0)
            break
    fh.close()


if __name__ == "__main__":
    main()
