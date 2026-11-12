"""
Integrated Webots controller: the whole valet agent in one loop.

    M1 sensors  ->  M2 slot choice + route (+ replanning)
                ->  M3 state machine (DRIVE, ALIGN, PARK, safety stops)
                ->  M4 path tracking + speed reference  ->  wheel and steering motors

All the decisions live in agent/valet_agent.py (tested offline with
`python -m agent.run_offline`). This file only reads the Webots devices,
calls agent.step() and sends the result to the motors.

To use it: set the valet_car's `controller` field to "valet_agent".

Devices (worlds/parking_lot.wbt, see docs/m1_world_sensors.md):
  gps, compass, lidar                      sensing (GPS at the rear-axle centre)
  {rear,front}_{left,right}_wheel_motor    velocity-controlled wheel motors
  front_{left,right}_steer                 position-controlled steering joints

How M4's output is applied: the wheel motors are velocity controlled, so
Webots' own motor controller holds the speed. We therefore send M4's speed
reference (v_ref, already ramped up and down by M4) and its steering angle,
and turn them into four wheel speeds and two Ackermann steering angles with
M1's tested helpers. M4's throttle/brake output (accel) is meant for a
torque-driven car and is not needed here.
"""

import math
import os
import sys

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from controller import Robot                                  # Webots API

from agent.valet_agent import AgentConfig, ValetAgent
from controllers.m1_actuator_check.m1_actuator_check import (STEER_LIMIT, ackermann_angles,
                                                             wheel_speeds)
from controllers.valet_controller.m1_sensors import compass_yaw, webots_ranges_to_ccw

LOG_PERIOD_S = 1.0
WHEELS = ("rear_left", "rear_right", "front_left", "front_right")


class Motors:
    """The car's four wheel motors and two steering motors."""

    def __init__(self, robot):
        self.wheels = [robot.getDevice(name + "_wheel_motor") for name in WHEELS]
        self.steer_left = robot.getDevice("front_left_steer")
        self.steer_right = robot.getDevice("front_right_steer")
        for motor in self.wheels:
            motor.setPosition(float("inf"))     # velocity control
            motor.setVelocity(0.0)
        self.steer_left.setPosition(0.0)
        self.steer_right.setPosition(0.0)

    def drive(self, speed, steer):
        """speed: signed rear-axle speed (m/s); steer: bicycle angle (rad, + = left)."""
        left, right = ackermann_angles(steer)
        self.steer_left.setPosition(max(-STEER_LIMIT, min(STEER_LIMIT, left)))
        self.steer_right.setPosition(max(-STEER_LIMIT, min(STEER_LIMIT, right)))
        for motor, omega in zip(self.wheels, wheel_speeds(speed, steer)):
            motor.setVelocity(omega)


def main():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    gps = robot.getDevice("gps")
    compass = robot.getDevice("compass")
    lidar = robot.getDevice("lidar")
    for device in (gps, compass, lidar):
        device.enable(timestep)
    motors = Motors(robot)

    agent = ValetAgent(config=AgentConfig(lidar_fov=lidar.getFov(),
                                          lidar_max_range=lidar.getMaxRange()))

    last_log, last_phase, logged_events = -LOG_PERIOD_S, None, 0
    while robot.step(timestep) != -1:
        now = robot.getTime()
        gx, gy, _ = gps.getValues()
        yaw = compass_yaw(compass.getValues())
        vx, vy, _ = gps.getSpeedVector()
        speed = vx * math.cos(yaw) + vy * math.sin(yaw)       # signed: + forward
        ranges = webots_ranges_to_ccw(lidar.getRangeImage())  # M1 functions expect CCW order

        act = agent.step(now, (gx, gy, yaw), speed, ranges)
        stop = act.brake_hold or act.hard_brake
        motors.drive(0.0 if stop else act.v_ref, act.steer)

        for t, text in agent.events[logged_events:]:
            print(f"[valet {t:7.2f}s] {text}", flush=True)
        logged_events = len(agent.events)
        if agent.phase != last_phase or now - last_log >= LOG_PERIOD_S:
            print(f"[valet {now:7.2f}s] {agent.phase:9s} pose=({gx:.2f}, {gy:.2f}, "
                  f"{math.degrees(yaw):.0f} deg) v={speed:+.2f} | {agent.reason}", flush=True)
            last_log, last_phase = now, agent.phase

        if agent.phase in ("DONE", "FAILED", "EMERGENCY_STOP") and abs(speed) < 0.05:
            print(f"[valet] finished: {agent.phase} - {agent.reason}", flush=True)
            motors.drive(0.0, 0.0)
            while robot.step(timestep) != -1:                 # stay stopped
                pass
            break


if __name__ == "__main__":
    main()
