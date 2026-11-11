"""M4 = the single owner of vehicle motion.  One call per simulation step:

    act = m4.step(m3_command, pose, speed, dt)

M3 (ParkingFSM) decides WHAT to do; M4 decides HOW:
  ControlMode.FOLLOW_ROUTE -> track M2's route (Pure Pursuit / Stanley + speed PID)
  ControlMode.PARKING      -> apply M3's target speed + steering, speed held by PID
  ControlMode.STOP         -> brake to standstill and hold (hard brake on EMERGENCY_STOP / FAILED)

`m4.route_complete` goes into ParkingObservation.route_complete.
Conventions: steering + = LEFT, speed + = forward, pose = rear-axle centre.
"""
from dataclasses import dataclass
import numpy as np

from behaviours.parking_types import ControlMode, ParkingState
from .follower import FollowerConfig, PathFollower
from .pid import PID


@dataclass
class Actuation:
    steer: float        # rad, + = left (convert to your car's API sign in the Webots layer)
    accel: float        # m/s^2 along the travel direction: + throttle, - brake
    direction: int      # +1 forward gear, -1 reverse gear
    v_ref: float        # signed speed reference [m/s]
    brake_hold: bool    # True -> apply parking brake (standing still)
    hard_brake: bool    # True -> emergency braking
    mode: str


class M4Controller:
    def __init__(self, route, lateral, max_steer, cfg=None, park_ramp_up=0.5, park_ramp_dn=1.5):
        self.cfg = cfg or FollowerConfig(max_steer=max_steer)
        self.max_steer = max_steer
        self.follower = PathFollower(route, lateral, self.cfg)
        c = self.cfg
        self.pid = PID(c.kp, c.ki, c.kd, -c.a_brk, c.a_acc)
        self.ramp_up, self.ramp_dn = park_ramp_up, park_ramp_dn
        self.vref = 0.0          # magnitude of the ramped speed reference
        self.direction = 1
        self.steer = 0.0
        self.cte = float('nan')      # route cross-track error (only while following the route)

    @property
    def route_complete(self):
        return self.follower.done

    def step(self, cmd, pose, v, dt):
        c = self.cfg

        # ---------------------------------------------------- follow M2's route
        if cmd.mode == ControlMode.FOLLOW_ROUTE:
            o = self.follower.step(pose.x, pose.y, pose.yaw, v, dt)
            self.steer, self.direction, self.cte = o.steer, o.direction, o.cte
            self.vref = abs(o.v_ref)
            self.pid.reset()
            return Actuation(o.steer, o.accel, o.direction, o.v_ref, o.brake_hold, False, "follow_route")

        # ---------------------------------------------------- parking / stop
        hard = cmd.state in (ParkingState.EMERGENCY_STOP, ParkingState.FAILED)
        if cmd.mode == ControlMode.PARKING and not hard:
            target = abs(cmd.target_speed_mps or 0.0)
            new_dir = 1 if (cmd.target_speed_mps or 0.0) >= 0 else -1
            if new_dir != self.direction and abs(v) < 0.05:     # M3 guarantees a stop first
                self.direction = new_dir
                self.pid.reset()
            if cmd.steering_rad is not None:
                self.steer = float(np.clip(cmd.steering_rad, -self.max_steer, self.max_steer))
            mode = "parking"
        else:
            target, mode = 0.0, ("emergency_stop" if hard else "stop")   # steering is held as is

        prev = self.vref
        if target > self.vref:
            self.vref = min(target, self.vref + self.ramp_up * dt)
        else:
            self.vref = max(target, self.vref - self.ramp_dn * dt)
        a_ff = (self.vref - prev) / dt

        if hard:
            self.vref = 0.0
            self.pid.reset()
            accel = -c.a_brk
        else:
            err = self.vref - self.direction * v
            accel = float(np.clip(self.pid.step(err, dt) + c.ff_gain * a_ff, -c.a_brk, c.a_acc))
        hold = (self.vref == 0.0) and abs(v) < 0.1
        return Actuation(self.steer, accel, self.direction, self.direction * self.vref, hold, hard, mode)
