"""Input checks, stopping distances and geometric parking checks."""

import math
from dataclasses import dataclass, fields

from .parking_types import ControlMode


def angle_error(a, b):
    return (a - b + math.pi) % (2 * math.pi) - math.pi


def finite(value):
    return isinstance(value, (int, float)) and math.isfinite(value)


@dataclass(frozen=True)
class SafetyConfig:
    """Initial simulation settings to tune with M1 and M4.

    The stopping-distance estimate assumes the configured braking
    deceleration is achievable by the selected vehicle.
    """

    clearance_margin_m: float = 0.15
    reaction_time_s: float = 0.2
    braking_deceleration_mps2: float = 1.0
    max_sensor_age_s: float = 0.2
    stopped_speed_mps: float = 0.03
    max_parking_speed_mps: float = 0.5
    arrival_distance_m: float = 0.6
    arrival_heading_rad: float = math.radians(12)
    parked_heading_rad: float = math.radians(3)
    parked_centre_tolerance_m: float = 0.15
    bay_inset_m: float = 0.05
    task_timeout_s: float = 180.0


class ParkingSafety:
    def __init__(self, vehicle, config=None):
        self.vehicle = vehicle
        self.config = config or SafetyConfig()

        for field in fields(self.config):
            value = getattr(self.config, field.name)
            if not finite(value) or value <= 0:
                raise ValueError("Safety settings must be finite and positive")

        values = [
            vehicle.wheelbase_m,
            vehicle.length_m,
            vehicle.width_m,
            vehicle.rear_overhang_m,
            vehicle.max_steer_rad,
        ]
        if not all(finite(value) for value in values):
            raise ValueError("Vehicle geometry must be finite")

        if (
            min(values[:3]) <= 0
            or vehicle.rear_overhang_m < 0
            or vehicle.length_m <= vehicle.wheelbase_m + vehicle.rear_overhang_m
            or not 0 < vehicle.max_steer_rad < math.pi / 2
        ):
            raise ValueError("Invalid vehicle dimensions or steering limit")

    def validate_target(self, slot, pre_park_pose):
        values = [
            slot.x,
            slot.y,
            slot.yaw,
            slot.width_m,
            slot.depth_m,
            pre_park_pose.x,
            pre_park_pose.y,
            pre_park_pose.yaw,
        ]
        if not all(finite(value) for value in values):
            raise ValueError("Slot and staging pose must be finite")

        margin = 2 * self.config.bay_inset_m
        if (
            slot.width_m <= self.vehicle.width_m + margin
            or slot.depth_m <= self.vehicle.length_m + margin
        ):
            raise ValueError("Slot is too small for this vehicle")

    def stopped(self, observation):
        return abs(observation.speed_mps) <= self.config.stopped_speed_mps

    def _motion_problem(self, observation, requested_speed=0.0):
        cfg = self.config
        gaps = [
            observation.front_clearance_m,
            observation.rear_clearance_m,
            observation.left_clearance_m,
            observation.right_clearance_m,
        ]
        if min(gaps) <= cfg.clearance_margin_m:
            return "Obstacle inside the minimum body clearance"

        speed = max(abs(observation.speed_mps), abs(requested_speed))
        required = (
            cfg.clearance_margin_m
            + speed * cfg.reaction_time_s
            + speed * speed / (2 * cfg.braking_deceleration_mps2)
        )

        if max(observation.speed_mps, requested_speed) > 0:
            if observation.front_clearance_m <= required:
                return "Insufficient front stopping distance"

        if min(observation.speed_mps, requested_speed) < 0:
            if observation.rear_clearance_m <= required:
                return "Insufficient rear stopping distance"

        return None

    def observation_error(self, observation, now_s):
        if not finite(now_s) or not finite(observation.timestamp_s):
            return "Invalid simulation timestamp"

        age = now_s - observation.timestamp_s
        if age < -1e-9 or age > self.config.max_sensor_age_s:
            return "Sensor snapshot is stale or from the future"

        if observation.pose is None:
            return "Car position or heading is unavailable"

        values = [
            observation.pose.x,
            observation.pose.y,
            observation.pose.yaw,
            observation.speed_mps,
        ]
        if not all(finite(value) for value in values):
            return "Invalid car position, heading or speed"

        gaps = [
            observation.front_clearance_m,
            observation.rear_clearance_m,
            observation.left_clearance_m,
            observation.right_clearance_m,
        ]
        if not all(finite(value) and value >= 0 for value in gaps):
            return "Missing or invalid obstacle clearances"

        if not isinstance(observation.route_complete, bool):
            return "Invalid route completion flag"

        return self._motion_problem(observation)

    def command_error(self, command):
        if command.mode == ControlMode.STOP:
            return None

        if command.mode != ControlMode.PARKING:
            return "Parking rules cannot request route-following mode"

        if not finite(command.target_speed_mps) or not finite(command.steering_rad):
            return "Parking speed and steering must be finite numbers"

        if abs(command.target_speed_mps) > self.config.max_parking_speed_mps:
            return "Parking speed exceeds the configured limit"

        if abs(command.steering_rad) > self.vehicle.max_steer_rad:
            return "Steering exceeds the vehicle limit"

        return None

    def motion_error(self, observation, command):
        speed = (
            command.target_speed_mps
            if command.mode == ControlMode.PARKING
            else 0.0
        )
        return self._motion_problem(observation, speed)

    def direction_change_requires_stop(self, observation, command):
        return (
            command.mode == ControlMode.PARKING
            and not self.stopped(observation)
            and observation.speed_mps * command.target_speed_mps < 0
        )

    def at_pre_park_pose(self, observation, goal):
        pose = observation.pose
        return (
            math.hypot(pose.x - goal.x, pose.y - goal.y)
            <= self.config.arrival_distance_m
            and abs(angle_error(pose.yaw, goal.yaw))
            <= self.config.arrival_heading_rad
        )

    def parked(self, observation, slot):
        if not self.stopped(observation):
            return False

        pose = observation.pose
        car = self.vehicle
        cfg = self.config

        if abs(angle_error(pose.yaw, slot.yaw)) > cfg.parked_heading_rad:
            return False

        c = math.cos(pose.yaw)
        s = math.sin(pose.yaw)

        centre_offset = car.length_m / 2 - car.rear_overhang_m
        centre_x = pose.x + centre_offset * c
        centre_y = pose.y + centre_offset * s

        if (
            math.hypot(centre_x - slot.x, centre_y - slot.y)
            > cfg.parked_centre_tolerance_m
        ):
            return False

        cs = math.cos(slot.yaw)
        ss = math.sin(slot.yaw)

        for along in (
            -car.rear_overhang_m,
            car.length_m - car.rear_overhang_m,
        ):
            for across in (-car.width_m / 2, car.width_m / 2):
                dx = pose.x + along * c - across * s - slot.x
                dy = pose.y + along * s + across * c - slot.y

                if (
                    abs(dx * cs + dy * ss)
                    > slot.depth_m / 2 - cfg.bay_inset_m
                    or abs(-dx * ss + dy * cs)
                    > slot.width_m / 2 - cfg.bay_inset_m
                ):
                    return False

        return True
