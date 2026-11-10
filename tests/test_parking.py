"""M3 state-machine and kinematic tests using Python's standard library.

Run from the repository root:
    python -m unittest discover -s tests -p "test_parking.py" -v

The motion model uses synthetic clearances. Webots collision testing with
the actual vehicle and parking lot is a separate integration step.
"""

import math
import unittest
from dataclasses import replace

from behaviours.parking_fsm import ParkingFSM
from behaviours.parking_manoeuvre import ManoeuvreConfig, ParkingManoeuvre
from behaviours.parking_types import (
    ControlMode, ParkingCommand, ParkingObservation, ParkingSlot,
    ParkingState, Pose, VehicleGeometry,
)


class FixedRules:
    def __init__(self, command):
        self.command = command

    def reset(self):
        pass

    def update(self, observation, slot, staging, now_s):
        return self.command


class ParkingFSMTests(unittest.TestCase):
    def setUp(self):
        self.car = VehicleGeometry(2.8, 4.6, 1.9, 1.0, 0.5)
        self.slot = ParkingSlot("A2", 10.0, 2.5, math.pi / 2, 2.5, 5.0)
        self.staging = Pose(13.0, 8.5, 0.0)
        self.observation = ParkingObservation(
            0.0, self.staging, 0.0, 10.0, 10.0, 10.0, 10.0
        )

    def machine(self, command=None):
        rules = FixedRules(command) if command is not None else None
        fsm = ParkingFSM(self.car, rules)
        fsm.select_slot(self.slot, self.staging)
        return fsm

    def reach_align(self, fsm):
        obs = replace(self.observation, route_complete=True)
        command = fsm.update(obs, 0.0)
        self.assertEqual(command.state, ParkingState.ALIGN)
        self.assertEqual(command.mode, ControlMode.STOP)
        return obs

    def test_waiting_for_target_holds_stop(self):
        fsm = ParkingFSM(self.car)
        command = fsm.update(self.observation, 0.0)
        self.assertEqual(command.state, ParkingState.SEARCH)
        self.assertEqual(command.mode, ControlMode.STOP)

    def test_drive_delegates_route_without_overriding_steering(self):
        command = self.machine().update(self.observation, 0.0)
        self.assertEqual(command.mode, ControlMode.FOLLOW_ROUTE)
        self.assertIsNone(command.steering_rad)
        self.assertIsNone(command.target_speed_mps)

    def test_route_completion_waits_until_car_stops(self):
        obs = replace(self.observation, route_complete=True, speed_mps=0.2)
        command = self.machine().update(obs, 0.0)
        self.assertEqual(command.state, ParkingState.DRIVE)
        self.assertEqual(command.mode, ControlMode.STOP)

    def test_wrong_route_endpoint_fails(self):
        obs = replace(self.observation, route_complete=True, pose=Pose(25.0, 8.5, 0.0))
        command = self.machine().update(obs, 0.0)
        self.assertEqual(command.state, ParkingState.FAILED)
        self.assertEqual(command.mode, ControlMode.STOP)

    def test_missing_rules_hold_stop(self):
        fsm = self.machine()
        obs = self.reach_align(fsm)
        command = fsm.update(obs, 0.0)
        self.assertEqual(command.state, ParkingState.ALIGN)
        self.assertEqual(command.mode, ControlMode.STOP)

    def test_stale_or_missing_sensors_latch_emergency_stop(self):
        bad_inputs = [replace(self.observation, timestamp_s=-1.0),
                      replace(self.observation, rear_clearance_m=None),
                      replace(self.observation, pose=None),
                      replace(self.observation, speed_mps=float("nan"))]
        for obs in bad_inputs:
            with self.subTest(observation=obs):
                fsm = self.machine()
                self.assertEqual(fsm.update(obs, 0.0).state, ParkingState.EMERGENCY_STOP)
                self.assertEqual(fsm.update(self.observation, 0.0).state,
                                 ParkingState.EMERGENCY_STOP)
                fsm.reset()
                self.assertEqual(fsm.state, ParkingState.SEARCH)
                self.assertIsNone(fsm.slot)

    def test_reverse_stopping_distance_is_checked_during_route(self):
        obs = replace(self.observation, speed_mps=-0.8, rear_clearance_m=0.3)
        command = self.machine().update(obs, 0.0)
        self.assertEqual(command.state, ParkingState.EMERGENCY_STOP)

    def test_local_request_is_checked_before_moving(self):
        request = ParkingCommand(ParkingState.PARK, ControlMode.PARKING, -0.5, -0.4)
        fsm = self.machine(request)
        obs = self.reach_align(fsm)
        command = fsm.update(replace(obs, rear_clearance_m=0.2), 0.0)
        self.assertEqual(command.state, ParkingState.EMERGENCY_STOP)
        self.assertEqual(command.mode, ControlMode.STOP)

    def test_direction_change_waits_for_stop(self):
        request = ParkingCommand(ParkingState.PARK, ControlMode.PARKING, -0.3, -0.4)
        fsm = self.machine(request)
        obs = self.reach_align(fsm)
        command = fsm.update(replace(obs, speed_mps=0.2), 0.0)
        self.assertEqual(command.mode, ControlMode.STOP)
        self.assertEqual(command.target_speed_mps, 0.0)
        command = fsm.update(obs, 0.0)
        self.assertEqual(command.mode, ControlMode.PARKING)
        self.assertLess(command.target_speed_mps, 0.0)

    def test_invalid_local_commands_fail_safely(self):
        bad_requests = [
            ParkingCommand(ParkingState.PARK, ControlMode.PARKING, -2.0, 0.1),
            ParkingCommand(ParkingState.PARK, ControlMode.PARKING, -0.3, 0.8),
            ParkingCommand(ParkingState.PARK, ControlMode.PARKING, float("nan"), 0.1),
            ParkingCommand(ParkingState.PARK, ControlMode.FOLLOW_ROUTE),
            None,
        ]
        for request in bad_requests:
            with self.subTest(command=request):
                fsm = ParkingFSM(self.car, FixedRules(request))
                fsm.select_slot(self.slot, self.staging)
                obs = self.reach_align(fsm)
                command = fsm.update(obs, 0.0)
                self.assertEqual(command.state, ParkingState.FAILED)
                self.assertEqual(command.mode, ControlMode.STOP)

    def test_success_requires_actual_body_and_heading_inside_slot(self):
        request = ParkingCommand(ParkingState.DONE, ControlMode.STOP)
        poses = [Pose(10.0, 1.2, math.pi / 2),
                 Pose(10.0, 2.5, math.pi / 2),
                 Pose(10.4, 1.2, math.pi / 2),
                 Pose(10.0, 1.2, 0.0)]
        for index, pose in enumerate(poses):
            with self.subTest(pose=pose):
                fsm = self.machine(request)
                obs = self.reach_align(fsm)
                command = fsm.update(replace(obs, pose=pose), 0.0)
                expected = ParkingState.DONE if index == 0 else ParkingState.FAILED
                self.assertEqual(command.state, expected)
                self.assertEqual(command.mode, ControlMode.STOP)

    def test_done_requires_stopped_car(self):
        fsm = self.machine(ParkingCommand(ParkingState.DONE, ControlMode.STOP))
        obs = self.reach_align(fsm)
        obs = replace(obs, pose=Pose(10.0, 1.2, math.pi / 2), speed_mps=-0.1)
        self.assertEqual(fsm.update(obs, 0.0).state, ParkingState.FAILED)

    def test_timeout_and_clock_reset_stop_the_task(self):
        fsm = self.machine()
        fsm.update(self.observation, 0.0)
        obs = replace(self.observation, timestamp_s=181.0)
        self.assertEqual(fsm.update(obs, 181.0).state, ParkingState.FAILED)
        fsm = self.machine()
        fsm.update(replace(self.observation, timestamp_s=2.0), 2.0)
        self.assertEqual(fsm.update(replace(self.observation, timestamp_s=1.0), 1.0).state,
                         ParkingState.EMERGENCY_STOP)


class ParkingMotionTests(unittest.TestCase):
    """Use a kinematic bicycle model with speed and steering rate limits.

    Constant clearances represent open space. These checks verify motion
    and supervisor integration; they do not replace Webots collision tests.
    """

    def simulate(self, heading=0.0, side=1, yaw_error=0.0, steering_rate=1.0,
                 obstacle_at=None, bump_m=0.0, max_corrections=2):
        car = VehicleGeometry(2.8, 4.6, 1.9, 1.0, 0.5)
        yaw = heading + side * math.pi / 2
        slot = ParkingSlot("test", 15.0, 15.0, yaw, 2.5, 5.0)
        sx = slot.x + 6 * math.cos(yaw) + 3 * math.cos(heading)
        sy = slot.y + 6 * math.sin(yaw) + 3 * math.sin(heading)
        staging = Pose(sx, sy, heading)
        pose = Pose(sx, sy, heading + yaw_error)
        rules = ParkingManoeuvre(car, ManoeuvreConfig(max_corrections=max_corrections))
        fsm = ParkingFSM(car, rules)
        fsm.select_slot(slot, staging)
        speed = steering = 0.0
        history = []
        bumped = False
        dt = 0.032
        for step in range(6000):
            t = step * dt
            if (not bumped and rules.phase == "reverse" and rules.path
                    and rules.path[-1][3] - rules.path[rules.index][3] < 0.4):
                pose = Pose(pose.x - bump_m * math.sin(yaw),
                            pose.y + bump_m * math.cos(yaw), pose.yaw)
                bumped = True
            rear = 0.1 if obstacle_at is not None and t >= obstacle_at else 10.0
            obs = ParkingObservation(t, pose, speed, 10.0, rear, 10.0, 10.0, True)
            command = fsm.update(obs, t)
            history.append((t, pose, speed, steering, command))
            if command.state in (ParkingState.DONE, ParkingState.FAILED,
                                 ParkingState.EMERGENCY_STOP):
                return command, pose, history, rules
            target_speed = command.target_speed_mps if command.mode == ControlMode.PARKING else 0.0
            target_steering = command.steering_rad if command.mode == ControlMode.PARKING else steering
            old_speed, old_steering = speed, steering
            speed += max(-dt, min(dt, target_speed - speed))
            steering += max(-steering_rate * dt,
                            min(steering_rate * dt, target_steering - steering))
            distance = (speed + old_speed) / 2 * dt
            delta = (steering + old_steering) / 2
            change = distance * math.tan(delta) / car.wheelbase_m
            middle = pose.yaw + change / 2
            pose = Pose(pose.x + distance * math.cos(middle),
                        pose.y + distance * math.sin(middle), pose.yaw + change)
        self.fail("Simulation never reached a terminal state")

    def test_reverse_parking_both_sides_and_approach_directions(self):
        for heading in (0.0, math.pi, 0.7):
            for side in (-1, 1):
                for error in (0.0, -0.15, 0.15):
                    with self.subTest(heading=heading, side=side, error=error):
                        command, pose, history, rules = self.simulate(
                            heading=heading, side=side, yaw_error=error)
                        self.assertEqual(command.state, ParkingState.DONE, command.reason)
                        self.assertEqual(command.mode, ControlMode.STOP)
                        self.assertLess(abs(history[-1][2]), 0.03)
                        self.assertLess(history[-1][0], 180.0)
                        # Fixture car: centre is 1.3 m ahead of its rear axle.
                        centre_x = pose.x + 1.3 * math.cos(pose.yaw)
                        centre_y = pose.y + 1.3 * math.sin(pose.yaw)
                        self.assertLess(math.hypot(centre_x - 15.0, centre_y - 15.0), 0.15)
                        expected_yaw = heading + side * math.pi / 2
                        error_rad = math.atan2(math.sin(pose.yaw - expected_yaw),
                                               math.cos(pose.yaw - expected_yaw))
                        self.assertLess(abs(error_rad), math.radians(3))
                        for _, _, _, _, request in history:
                            if request.mode == ControlMode.PARKING:
                                self.assertLessEqual(abs(request.target_speed_mps), 0.5)
                                self.assertLessEqual(abs(request.steering_rad), 0.5)

    def test_slower_steering_response_still_parks(self):
        for side in (-1, 1):
            with self.subTest(side=side):
                command, _, _, _ = self.simulate(side=side, steering_rate=0.2)
                self.assertEqual(command.state, ParkingState.DONE, command.reason)

    def test_obstacle_interrupts_setup_turn_and_final_reverse(self):
        for t in (1.0, 15.0, 40.0):
            with self.subTest(time=t):
                command, _, _, _ = self.simulate(obstacle_at=t)
                self.assertEqual(command.state, ParkingState.EMERGENCY_STOP)
                self.assertEqual(command.mode, ControlMode.STOP)

    def test_correction_recovers_a_lateral_disturbance(self):
        for bump in (-0.3, -0.18, 0.18, 0.3):
            with self.subTest(bump=bump):
                command, _, _, rules = self.simulate(bump_m=bump)
                self.assertEqual(command.state, ParkingState.DONE, command.reason)
                self.assertGreater(rules.corrections, 0)
                self.assertLessEqual(rules.corrections, 2)

    def test_corrections_can_be_disabled(self):
        command, _, _, rules = self.simulate(bump_m=0.3, max_corrections=0)
        self.assertEqual(command.state, ParkingState.FAILED)
        self.assertEqual(command.mode, ControlMode.STOP)
        self.assertEqual(rules.corrections, 0)

    def test_tight_or_nonperpendicular_staging_is_rejected(self):
        car = VehicleGeometry(2.8, 4.6, 1.9, 1.0, 0.5)
        slot = ParkingSlot("test", 15.0, 15.0, math.pi / 2, 2.5, 5.0)
        for staging in (Pose(18.0, 18.0, 0.0), Pose(18.0, 21.0, 0.3)):
            with self.subTest(staging=staging):
                obs = ParkingObservation(0.0, staging, 0.0, 10.0, 10.0, 10.0, 10.0, True)
                command = ParkingManoeuvre(car).update(obs, slot, staging, 0.0)
                self.assertEqual(command.state, ParkingState.FAILED)
                self.assertEqual(command.mode, ControlMode.STOP)


if __name__ == "__main__":
    unittest.main()
