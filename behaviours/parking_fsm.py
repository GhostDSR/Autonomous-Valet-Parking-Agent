"""Supervise route following, parking manoeuvres and stopping."""

from .parking_types import ControlMode, ParkingCommand, ParkingState
from .parking_safety import ParkingSafety


class ParkingFSM:
    """Call update once per simulation step; send its result to M4.

    manoeuvre is the parking-rules object that we connect separately.
    now_s must be the current Webots simulation time, in seconds.
    Emergency stops and failures remain latched until reset is called.
    """

    def __init__(self, vehicle, manoeuvre=None, safety_config=None):
        self.safety = ParkingSafety(vehicle, safety_config)
        self.manoeuvre = manoeuvre
        self.reset()

    def reset(self):
        """Explicitly abandon the current task and return to SEARCH."""
        self.state = ParkingState.SEARCH
        self.slot = None
        self.pre_park_pose = None
        self.started_s = None
        self.last_now_s = None
        self.reason = "Waiting for a free slot"
        if self.manoeuvre is not None:
            self.manoeuvre.reset()

    def select_slot(self, slot, pre_park_pose):
        """Use the free slot from M1 and the chosen staging pose from M2.

        Call this after M2 finds a route. M4 must receive that same route.
        """
        if self.state != ParkingState.SEARCH:
            raise RuntimeError("Reset before selecting another slot")
        self.safety.validate_target(slot, pre_park_pose)
        self.slot = slot
        self.pre_park_pose = pre_park_pose
        self.state = ParkingState.DRIVE
        self.reason = "Follow the route to the pre-parking pose"

    def _stop(self, reason):
        self.reason = reason
        return ParkingCommand(
            state=self.state,
            mode=ControlMode.STOP,
            target_speed_mps=0.0,
            reason=reason,
        )

    def _finish(self, state, reason):
        self.state = state
        return self._stop(reason)

    def update(self, observation, now_s):
        """Return one command using the latest sensor snapshot."""
        if self.state in (
            ParkingState.DONE,
            ParkingState.FAILED,
            ParkingState.EMERGENCY_STOP,
        ):
            return self._stop(self.reason)

        if self.slot is None:
            return self._stop("Waiting for a free slot and a route")

        problem = self.safety.observation_error(observation, now_s)
        if problem:
            return self._finish(ParkingState.EMERGENCY_STOP, problem)
        if self.last_now_s is not None and now_s < self.last_now_s:
            return self._finish(
                ParkingState.EMERGENCY_STOP, "Simulation time moved backwards"
            )
        self.last_now_s = now_s
        if self.started_s is None:
            self.started_s = now_s
        if now_s - self.started_s > self.safety.config.task_timeout_s:
            return self._finish(ParkingState.FAILED, "Parking task timed out")

        if self.state == ParkingState.DRIVE:
            if not observation.route_complete:
                return ParkingCommand(
                    state=self.state,
                    mode=ControlMode.FOLLOW_ROUTE,
                    reason="M4 follows the route supplied by M2",
                )
            if not self.safety.stopped(observation):
                return self._stop("Stop at the end of the route")
            if not self.safety.at_pre_park_pose(observation, self.pre_park_pose):
                return self._finish(
                    ParkingState.FAILED, "Route ended away from its staging pose"
                )
            self.state = ParkingState.ALIGN
            return self._stop("Route complete; prepare the parking manoeuvre")

        if self.manoeuvre is None:
            return self._stop("Waiting for the parking-rules module")

        try:
            command = self.manoeuvre.update(
                observation, self.slot, self.pre_park_pose, now_s
            )
            if not isinstance(command, ParkingCommand):
                raise ValueError("Parking rules must return ParkingCommand")
            allowed = {
                ParkingState.ALIGN, ParkingState.PARK, ParkingState.DONE,
                ParkingState.FAILED, ParkingState.EMERGENCY_STOP,
            }
            if command.state not in allowed:
                raise ValueError("Parking rules returned an invalid state")
            if self.state == ParkingState.PARK and command.state == ParkingState.ALIGN:
                raise ValueError("Parking rules cannot return to ALIGN")
            if command.state == ParkingState.DONE:
                if not self.safety.parked(observation, self.slot):
                    raise ValueError("Car is not stopped and correctly inside the bay")
                return self._finish(ParkingState.DONE, "Parking completed")
            if command.state in (ParkingState.FAILED, ParkingState.EMERGENCY_STOP):
                return self._finish(command.state, command.reason)
            problem = self.safety.command_error(command)
            if problem:
                raise ValueError(problem)
        except Exception as error:
            return self._finish(ParkingState.FAILED, "Parking rules error: " + str(error))

        self.state = command.state
        problem = self.safety.motion_error(observation, command)
        if problem:
            return self._finish(ParkingState.EMERGENCY_STOP, problem)
        if self.safety.direction_change_requires_stop(observation, command):
            return self._stop("Stop before changing travel direction")
        if command.mode == ControlMode.STOP:
            return self._stop(command.reason)
        self.reason = command.reason
        return command
