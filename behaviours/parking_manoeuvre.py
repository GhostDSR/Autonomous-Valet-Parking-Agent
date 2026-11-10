"""Reactive perpendicular reverse parking for an Ackermann vehicle.

Phases: move forward to a feasible turning pose, stop, reverse along a
quarter-circle and straight segment, then stop and check the whole car.
Up to two short forward/reverse corrections are allowed. Use through
ParkingFSM so emergency stops remain latched. Validate and tune in Webots.

Steering uses the pure-pursuit geometry described by R. C. Coulter,
CMU-RI-TR-92-01 (1992). Positive steering is LEFT, as in parking_types.py.
"""

import math
from dataclasses import dataclass, fields

from .parking_safety import ParkingSafety, angle_error, finite
from .parking_types import ControlMode, ParkingCommand, ParkingState


@dataclass(frozen=True)
class ManoeuvreConfig:
    speed_mps: float = 0.35
    lookahead_m: float = 0.65
    sample_step_m: float = 0.05
    position_tolerance_m: float = 0.06
    tracking_limit_m: float = 0.6
    steering_fraction: float = 0.85
    minimum_setup_m: float = 0.8
    minimum_straight_m: float = 0.5
    correction_distance_m: float = 2.5
    phase_timeout_s: float = 70.0
    max_corrections: int = 2


class ParkingManoeuvre:
    """Supply this object as manoeuvre=... when constructing ParkingFSM.

    Supports a perpendicular bay on either side of the aisle and both
    approach directions. Clearances must include body corners. They check
    current clearance; they do not certify an entire future swept path.
    """

    def __init__(self, vehicle, config=None, safety_config=None):
        self.safety = ParkingSafety(vehicle, safety_config)
        self.vehicle = vehicle
        self.config = config or ManoeuvreConfig()
        for field in fields(self.config):
            value = getattr(self.config, field.name)
            if not finite(value) or value < 0:
                raise ValueError("Manoeuvre settings must be finite and nonnegative")
            if field.name != "max_corrections" and value == 0:
                raise ValueError("Manoeuvre distances, speed and times must be positive")
        if not 0 < self.config.steering_fraction < 1:
            raise ValueError("steering_fraction must be between zero and one")
        if type(self.config.max_corrections) is not int:
            raise ValueError("max_corrections must be an integer")
        if self.config.speed_mps > self.safety.config.max_parking_speed_mps:
            raise ValueError("Manoeuvre speed exceeds the safety limit")
        self.reset()

    def reset(self):
        self.phase = "setup"
        self.target_key = None
        self.corrections = 0
        self.phase_started_s = None
        self.last_now_s = None
        self.path = []
        self.index = 0
        self.failure = ""

    def _command(self, state, reason, speed=None, steering=None):
        mode = ControlMode.STOP if speed is None else ControlMode.PARKING
        return ParkingCommand(state, mode, 0.0 if speed is None else speed,
                              steering, reason)

    def _fail(self, reason):
        self.phase = "failed"
        self.failure = reason
        return self._command(ParkingState.FAILED, reason)

    def _line(self, start, end, yaw):
        distance = math.hypot(end[0] - start[0], end[1] - start[1])
        count = max(1, math.ceil(distance / self.config.sample_step_m))
        return [(start[0] + (end[0] - start[0]) * i / count,
                 start[1] + (end[1] - start[1]) * i / count, yaw)
                for i in range(count + 1)]

    def _centre_line(self, pose, end, yaw):
        along = ((pose.x - self.goal[0]) * self.h[0]
                 + (pose.y - self.goal[1]) * self.h[1])
        start = (self.goal[0] + along * self.h[0],
                 self.goal[1] + along * self.h[1])
        return self._line(start, end, yaw)

    def _begin(self, phase, points, now_s):
        self.phase, self.index = phase, 0
        self.phase_started_s = now_s
        self.path = []
        distance = 0.0
        for x, y, yaw in points:
            if self.path:
                distance += math.hypot(x - self.path[-1][0], y - self.path[-1][1])
            self.path.append((x, y, yaw, distance))

    def _setup(self, observation, slot, staging, now_s):
        self.safety.validate_target(slot, staging)
        delta = angle_error(slot.yaw, staging.yaw)
        if abs(abs(delta) - math.pi / 2) > math.radians(0.5):
            raise ValueError("This manoeuvre requires a perpendicular bay")
        turn = 1 if delta > 0 else -1
        approach_yaw = slot.yaw - turn * math.pi / 2
        self.f = (math.cos(approach_yaw), math.sin(approach_yaw))
        self.h = (math.cos(slot.yaw), math.sin(slot.yaw))
        offset = self.vehicle.length_m / 2 - self.vehicle.rear_overhang_m
        self.goal = (slot.x - offset * self.h[0], slot.y - offset * self.h[1])
        pose = observation.pose
        dx, dy = pose.x - self.goal[0], pose.y - self.goal[1]
        along = dx * self.f[0] + dy * self.f[1]
        depth = dx * self.h[0] + dy * self.h[1]
        steer = self.vehicle.max_steer_rad * self.config.steering_fraction
        radius = max(self.vehicle.wheelbase_m / math.tan(steer),
                     along + self.config.minimum_setup_m)
        if depth - radius < self.config.minimum_straight_m:
            raise ValueError("Insufficient turning room; request another staging pose")
        self.stage = (self.goal[0] + radius * self.f[0] + depth * self.h[0],
                      self.goal[1] + radius * self.f[1] + depth * self.h[1])
        count = math.ceil(radius * math.pi / (2 * self.config.sample_step_m))
        self.reverse_points = []
        for i in range(count + 1):
            u = math.pi * i / (2 * count)
            x = (self.stage[0] - radius * math.sin(u) * self.f[0]
                 - radius * (1 - math.cos(u)) * self.h[0])
            y = (self.stage[1] - radius * math.sin(u) * self.f[1]
                 - radius * (1 - math.cos(u)) * self.h[1])
            self.reverse_points.append((x, y, approach_yaw + turn * u))
        end = self.reverse_points[-1]
        self.reverse_points += self._line(end, self.goal, slot.yaw)[1:]
        self._begin("align", self._line((pose.x, pose.y), self.stage, approach_yaw), now_s)

    def _lookahead(self, distance, direction):
        for point in self.path[self.index:]:
            if point[3] >= distance:
                return point[0], point[1]
        x, y, yaw, end_s = self.path[-1]
        extra = distance - end_s
        return (x + direction * extra * math.cos(yaw),
                y + direction * extra * math.sin(yaw))

    def _follow(self, observation, direction, state):
        pose = observation.pose
        window = max(2, math.ceil(1.0 / self.config.sample_step_m))
        stop = min(len(self.path), self.index + window + 1)
        self.index = min(range(self.index, stop), key=lambda i:
                         (self.path[i][0] - pose.x) ** 2
                         + (self.path[i][1] - pose.y) ** 2)
        nearest = self.path[self.index]
        if math.hypot(nearest[0] - pose.x, nearest[1] - pose.y) > self.config.tracking_limit_m:
            return self._fail("Car left the parking path; stop and replan")
        gx, gy, gyaw, total = self.path[-1]
        remaining = direction * ((gx - pose.x) * math.cos(gyaw)
                                 + (gy - pose.y) * math.sin(gyaw))
        near_end = total - nearest[3] < self.config.lookahead_m
        if near_end and remaining <= self.config.position_tolerance_m:
            return None
        tx, ty = self._lookahead(nearest[3] + self.config.lookahead_m, direction)
        dx, dy = tx - pose.x, ty - pose.y
        body_y = -math.sin(pose.yaw) * dx + math.cos(pose.yaw) * dy
        steering = math.atan2(2 * self.vehicle.wheelbase_m * body_y,
                              max(dx * dx + dy * dy, 1e-9))
        steering = max(-self.vehicle.max_steer_rad,
                       min(self.vehicle.max_steer_rad, steering))
        distance_left = max(0.0, total - nearest[3])
        speed = min(self.config.speed_mps, max(0.05, 0.8 * distance_left))
        reason = {"align": "Move forward to the turning pose",
                  "reverse": "Reverse turn, then straighten into the bay",
                  "correct": "Move forward for an alignment correction"}[self.phase]
        return self._command(state, reason, direction * speed, steering)

    def update(self, observation, slot, pre_park_pose, now_s):
        """Return a movement request; ParkingFSM applies the safety gates."""
        if self.phase == "failed":
            return self._command(ParkingState.FAILED, self.failure)
        if self.phase == "done":
            return self._command(ParkingState.DONE, "Parking completed")
        problem = self.safety.observation_error(observation, now_s)
        if problem:
            return self._command(ParkingState.EMERGENCY_STOP, problem)
        if self.last_now_s is not None and now_s < self.last_now_s:
            return self._fail("Simulation time moved backwards")
        self.last_now_s = now_s
        key = (slot, pre_park_pose)
        if self.target_key is not None and key != self.target_key:
            return self._fail("Parking target changed; reset and replan")
        if self.phase == "setup":
            self.target_key = key
            try:
                self._setup(observation, slot, pre_park_pose, now_s)
            except ValueError as error:
                return self._fail(str(error))
        if now_s - self.phase_started_s > self.config.phase_timeout_s:
            return self._fail("Parking manoeuvre phase timed out")

        state = ParkingState.ALIGN if self.phase == "align" else ParkingState.PARK
        direction = -1 if self.phase == "reverse" else 1
        command = self._follow(observation, direction, state)
        if command is not None:
            return command
        if not self.safety.stopped(observation):
            return self._command(state, "Stop before the next parking phase")

        pose = observation.pose
        gx, gy, gyaw, _ = self.path[-1]
        position_ok = math.hypot(gx - pose.x, gy - pose.y) <= 0.12
        heading_ok = abs(angle_error(pose.yaw, gyaw)) <= math.radians(3)
        if self.phase == "align":
            if not position_ok or not heading_ok:
                return self._fail("Starting alignment needs a new staging pose")
            self._begin("reverse", self.reverse_points, now_s)
            return self._command(ParkingState.PARK, "Aligned; prepare to reverse")
        if self.phase == "correct":
            points = self._centre_line(pose, self.goal, slot.yaw)
            self._begin("reverse", points, now_s)
            return self._command(ParkingState.PARK, "Correction complete; prepare to reverse")
        if self.safety.parked(observation, slot):
            self.phase = "done"
            return self._command(ParkingState.DONE, "Stopped and correctly inside the bay")
        if self.corrections >= self.config.max_corrections:
            return self._fail("Alignment correction limit reached")
        self.corrections += 1
        end = (self.goal[0] + self.config.correction_distance_m * self.h[0],
               self.goal[1] + self.config.correction_distance_m * self.h[1])
        self._begin("correct", self._centre_line(pose, end, slot.yaw), now_s)
        return self._command(ParkingState.PARK, "Stop; prepare a forward correction")
