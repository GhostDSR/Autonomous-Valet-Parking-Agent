"""M1 Webots sensor bring-up controller.

This controller is deliberately sensor-only: M4 is the single owner of
vehicle commands. It enables the M1 devices, publishes pose/clearances and
slot classifications, and holds the Ackermann actuators at zero.
"""
import json
import math
import os
import sys

from controller import Robot

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from controllers.valet_controller.m1_sensors import (
    body_clearances_from_scan,
    classify_slots_from_scan,
    compass_yaw,
    lidar_points_world,
)


def main():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    gps = robot.getDevice("gps")
    compass = robot.getDevice("compass")
    imu = robot.getDevice("imu")
    lidar = robot.getDevice("lidar")
    for device in (gps, compass, imu, lidar):
        device.enable(timestep)

    distance_sensors = {}
    for name in ("ds_front", "ds_rear", "ds_left", "ds_right"):
        sensor = robot.getDevice(name)
        sensor.enable(timestep)
        distance_sensors[name] = sensor

    # M1 defines the Ackermann hardware. M4 will later own the motion loop.
    for name in ("front_left_steer", "front_right_steer"):
        motor = robot.getDevice(name)
        motor.setPosition(0.0)
        motor.setVelocity(1.0)
    for name in ("front_left_wheel_motor", "front_right_wheel_motor",
                 "rear_left_wheel_motor", "rear_right_wheel_motor"):
        motor = robot.getDevice(name)
        motor.setPosition(float("inf"))
        motor.setVelocity(0.0)

    last_report = -1.0
    while robot.step(timestep) != -1:
        now = robot.getTime()
        if now - last_report < 1.0:
            continue
        last_report = now

        gps_values = gps.getValues()
        yaw = compass_yaw(compass.getValues())
        pose = (float(gps_values[0]), float(gps_values[1]), yaw)
        ranges = lidar.getRangeImage()
        fov = 2.0 * math.pi
        max_range = float(lidar.getMaxRange())

        occupancy = classify_slots_from_scan(
            ranges, pose, field_of_view=fov, max_range=max_range
        )
        clearances = body_clearances_from_scan(
            ranges, pose, field_of_view=fov
        )
        points = lidar_points_world(
            ranges, pose, field_of_view=fov, max_range=max_range
        )
        report = {
            "time_s": round(now, 3),
            "pose_rear_axle": [round(v, 4) for v in pose],
            "speed_mps": 0.0,
            "clearance_m": {
                key: None if value is None else round(value, 3)
                for key, value in clearances.items()
            },
            "slots": occupancy,
            "slot_counts": {
                "occupied": sum(v == "occupied" for v in occupancy.values()),
                "free": sum(v == "free" for v in occupancy.values()),
                "unknown": sum(v == "unknown" for v in occupancy.values()),
            },
            "lidar_points": len(points),
            "distance_sensor_raw": {
                name: round(sensor.getValue(), 3)
                for name, sensor in distance_sensors.items()
            },
        }
        print("M1_SENSOR " + json.dumps(report, sort_keys=True), flush=True)


if __name__ == "__main__":
    main()
