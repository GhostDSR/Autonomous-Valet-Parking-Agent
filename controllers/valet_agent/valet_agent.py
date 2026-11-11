"""
Integrated Webots controller: the whole valet agent in one loop.

    M1 sensors  ->  M2 slot choice + route (+ replanning)
                ->  M3 state machine (DRIVE, ALIGN, PARK, safety stops)
                ->  M4 path tracking + speed control  ->  car

All the decisions live in agent/valet_agent.py (tested offline with
`python -m agent.run_offline`). This file only reads the Webots devices,
calls agent.step() and sends the result to the car.

To use it: set the car's `controller` field to "valet_agent".

Requirements on the world (see docs/integration.md):
  - the car is a Webots `Car` / `AckermannVehicle`, so the `vehicle.Driver`
    API (steering angle, throttle, brake, gear) is available;
  - devices named "gps", "compass" and "lidar" (360-degree, single layer),
    with the GPS at the rear-axle centre (the robot's origin in our world).
"""

import math
import os
import sys

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from vehicle import Driver                                   # Webots automobile library

from agent.valet_agent import AgentConfig, ValetAgent
from m4_control.webots_controller import CONFIG as M4_CONFIG, DriverCar

LOG_PERIOD_S = 1.0


def heading_from_compass(north):
    """
    Car heading (anticlockwise from world +x) from the Webots compass.

    The compass gives the direction of world north (+y) in the car's own
    frame (x forward, y left). For a car with heading yaw that vector is
    (sin yaw, cos yaw), so yaw = atan2(n_x, n_y). (M1's compass_yaw uses
    atan2(-n_x, n_y), which turns the wrong way; M4's adapter agrees with
    this one. Confirm once with M4's calibrate mode in Webots.)
    """
    return math.atan2(north[0], north[1])


def main():
    driver = Driver()
    timestep = int(driver.getBasicTimeStep())

    gps = driver.getDevice("gps")
    compass = driver.getDevice("compass")
    lidar = driver.getDevice("lidar")
    for device in (gps, compass, lidar):
        device.enable(timestep)

    agent = ValetAgent(config=AgentConfig(lidar_fov=lidar.getFov(),
                                          lidar_max_range=lidar.getMaxRange()))
    car = DriverCar(driver, M4_CONFIG, agent.follower_cfg)    # M4's actuation adapter

    last_log, last_phase, logged_events = -LOG_PERIOD_S, None, 0
    while driver.step() != -1:
        now = driver.getTime()
        gx, gy, _ = gps.getValues()
        yaw = heading_from_compass(compass.getValues())
        vx, vy, _ = gps.getSpeedVector()
        speed = vx * math.cos(yaw) + vy * math.sin(yaw)       # signed: + forward
        ranges = lidar.getRangeImage()

        actuation = agent.step(now, (gx, gy, yaw), speed, ranges)
        car.apply(actuation, speed)

        for t, text in agent.events[logged_events:]:
            print(f"[valet {t:7.2f}s] {text}", flush=True)
        logged_events = len(agent.events)
        if agent.phase != last_phase or now - last_log >= LOG_PERIOD_S:
            print(f"[valet {now:7.2f}s] {agent.phase:9s} pose=({gx:.2f}, {gy:.2f}, "
                  f"{math.degrees(yaw):.0f} deg) v={speed:+.2f} | {agent.reason}", flush=True)
            last_log, last_phase = now, agent.phase

        if agent.phase in ("DONE", "FAILED", "EMERGENCY_STOP") and abs(speed) < 0.05:
            print(f"[valet] finished: {agent.phase} - {agent.reason}", flush=True)
            while driver.step() != -1:                        # hold the brakes
                driver.setThrottle(0.0)
                driver.setBrakeIntensity(1.0)
            break


if __name__ == "__main__":
    main()
