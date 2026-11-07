"""Shared inputs and outputs for the M3 parking controller.

Use metres, seconds and radians. Speed is positive forward and negative
in reverse. Heading increases anticlockwise from world +x. Steering is
positive left in this module; M4 converts it to the vehicle API convention.
"""

from dataclasses import dataclass
from enum import Enum
from typing import Optional


class ParkingState(str, Enum):
    SEARCH = "search"
    DRIVE = "drive"
    ALIGN = "align"
    PARK = "park"
    DONE = "done"
    EMERGENCY_STOP = "emergency_stop"
    FAILED = "failed"


class ControlMode(str, Enum):
    FOLLOW_ROUTE = "follow_route"
    PARKING = "parking"
    STOP = "stop"


@dataclass(frozen=True)
class Pose:
    """Car pose at the centre of the rear axle."""

    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class ParkingSlot:
    """Bay centre, desired parked-car heading, and bay dimensions."""

    slot_id: str
    x: float
    y: float
    yaw: float
    width_m: float
    depth_m: float


@dataclass(frozen=True)
class VehicleGeometry:
    """Actual vehicle dimensions supplied by M1."""

    wheelbase_m: float
    length_m: float
    width_m: float
    rear_overhang_m: float
    max_steer_rad: float


@dataclass(frozen=True)
class ParkingObservation:
    """One sensor snapshot; None means unavailable or invalid.

    timestamp_s is the simulation time when the snapshot was measured.
    Clearances are distances from the car body to the nearest obstacle
    in each region, including the corners, rather than raw sensor ranges.
    route_complete is M4's indication that route following has finished.
    """

    timestamp_s: float
    pose: Optional[Pose]
    speed_mps: Optional[float]
    front_clearance_m: Optional[float]
    rear_clearance_m: Optional[float]
    left_clearance_m: Optional[float]
    right_clearance_m: Optional[float]
    route_complete: bool = False


@dataclass(frozen=True)
class ParkingCommand:
    """Request to M4; M4 alone applies vehicle commands.

    FOLLOW_ROUTE: M4 tracks M2's route and handles speed and steering.
    PARKING: M4 uses the supplied target speed and steering angle.
    STOP: M4 overrides motion requests and brakes to a stop, then holds.
    None means no override; it is not permission to move in STOP mode.
    """

    state: ParkingState
    mode: ControlMode
    target_speed_mps: Optional[float] = None
    steering_rad: Optional[float] = None
    reason: str = ""
