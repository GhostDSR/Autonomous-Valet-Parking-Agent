"""M1 sensor geometry and LiDAR-based parking-slot occupancy detection.

All coordinates are Webots world XY metres. Heading is anticlockwise from +X.
The robot pose is the centre of the rear axle, matching M2 and M3.
"""
from __future__ import annotations

import math
from typing import Dict, Iterable, List, Sequence, Tuple

from planner.parking_lot import make_slots, PARKED_CAR_LENGTH, PARKED_CAR_WIDTH

Point = Tuple[float, float]
Pose2D = Tuple[float, float, float]


def lidar_points_world(
    ranges: Sequence[float],
    pose: Pose2D,
    field_of_view: float = 2.0 * math.pi,
    max_range: float = 40.0,
    min_range: float = 0.05,
) -> List[Point]:
    """Convert a planar, evenly sampled LiDAR range image into world XY points.

    Webots range-image samples are ordered from -FOV/2 to +FOV/2.
    Invalid, infinite, and max-range/no-return samples are discarded.
    """
    x, y, yaw = pose
    if len(ranges) < 2:
        return []
    points = []
    step = field_of_view / (len(ranges) - 1)
    for index, distance in enumerate(ranges):
        try:
            distance = float(distance)
        except (TypeError, ValueError):
            continue
        if not math.isfinite(distance) or distance < min_range or distance >= max_range:
            continue
        angle = -field_of_view / 2.0 + index * step + yaw
        points.append((x + distance * math.cos(angle), y + distance * math.sin(angle)))
    return points


def _ray_rectangle_interval(
    origin: Point,
    direction: Point,
    centre: Point,
    yaw: float,
    half_length: float,
    half_width: float,
):
    """Return ray entry/exit distances through an oriented rectangle, or None."""
    dx, dy = direction
    ox, oy = origin[0] - centre[0], origin[1] - centre[1]
    c, s = math.cos(yaw), math.sin(yaw)
    # Transform ray origin and direction into the rectangle's local frame.
    local_o = (c * ox + s * oy, -s * ox + c * oy)
    local_d = (c * dx + s * dy, -s * dx + c * dy)
    t_min, t_max = 0.0, float("inf")
    for o, d, half in ((local_o[0], local_d[0], half_length),
                       (local_o[1], local_d[1], half_width)):
        if abs(d) < 1e-10:
            if o < -half or o > half:
                return None
            continue
        a, b = (-half - o) / d, (half - o) / d
        if a > b:
            a, b = b, a
        t_min, t_max = max(t_min, a), min(t_max, b)
        if t_min > t_max:
            return None
    if t_max < 0:
        return None
    return max(0.0, t_min), t_max


def classify_slots_from_scan(
    ranges: Sequence[float],
    pose: Pose2D,
    slots=None,
    field_of_view: float = 2.0 * math.pi,
    max_range: float = 40.0,
    visibility_margin: float = 0.2,
    occupancy_padding: float = 0.2,
) -> Dict[str, str]:
    """Classify each bay as occupied, free, or unknown using LiDAR ray evidence.

    A ray that reaches beyond a bay proves that the bay was visible and clear
    along that ray. A return within the padded parked-car footprint indicates
    occupancy. Bays hidden behind another object remain unknown rather than
    being incorrectly advertised as free.
    """
    slots = make_slots() if slots is None else slots
    x, y, yaw = pose
    result = {slot_id: "unknown" for slot_id in slots}
    visible = {slot_id: False for slot_id in slots}
    occupied = {slot_id: False for slot_id in slots}
    if len(ranges) < 2:
        return result

    step = field_of_view / (len(ranges) - 1)
    half_length = PARKED_CAR_LENGTH / 2.0 + occupancy_padding
    half_width = PARKED_CAR_WIDTH / 2.0 + occupancy_padding

    for index, raw_distance in enumerate(ranges):
        try:
            distance = float(raw_distance)
        except (TypeError, ValueError):
            continue
        if not math.isfinite(distance) or distance <= 0 or distance >= max_range:
            continue
        angle = -field_of_view / 2.0 + index * step + yaw
        direction = (math.cos(angle), math.sin(angle))
        for slot_id, slot in slots.items():
            interval = _ray_rectangle_interval(
                (x, y), direction, (slot.x, slot.y), slot.yaw,
                half_length, half_width,
            )
            if interval is None:
                continue
            entry, exit_distance = interval
            if distance >= entry - visibility_margin:
                visible[slot_id] = True
            if entry - visibility_margin <= distance <= exit_distance + visibility_margin:
                occupied[slot_id] = True

    for slot_id in slots:
        if occupied[slot_id]:
            result[slot_id] = "occupied"
        elif visible[slot_id]:
            result[slot_id] = "free"
    return result


def body_clearances_from_scan(
    ranges: Sequence[float],
    pose: Pose2D,
    field_of_view: float = 2.0 * math.pi,
    front_extent: float = 3.6,
    rear_extent: float = 1.0,
    half_width: float = 0.95,
    sector_half_angle: float = math.radians(25.0),
) -> Dict[str, float | None]:
    """Estimate front/rear/left/right body clearances from a 360-degree scan.

    The LiDAR origin is at the rear-axle reference point. The support distance
    of the rectangular vehicle footprint is subtracted from each ray range.
    A missing sector returns None; callers must not invent a large clearance.
    """
    if len(ranges) < 2:
        return {"front": None, "rear": None, "left": None, "right": None}
    sectors = {"front": 0.0, "left": math.pi / 2,
               "rear": math.pi, "right": -math.pi / 2}
    minima = {name: None for name in sectors}
    step = field_of_view / (len(ranges) - 1)
    for index, raw_distance in enumerate(ranges):
        try:
            distance = float(raw_distance)
        except (TypeError, ValueError):
            continue
        if not math.isfinite(distance) or distance <= 0:
            continue
        local_angle = -field_of_view / 2.0 + index * step
        for name, centre_angle in sectors.items():
            delta = (local_angle - centre_angle + math.pi) % (2 * math.pi) - math.pi
            if abs(delta) > sector_half_angle:
                continue
            c, s = math.cos(local_angle), math.sin(local_angle)
            support_x = front_extent if c >= 0 else rear_extent
            support = support_x * abs(c) + half_width * abs(s)
            clearance = max(0.0, distance - support)
            if minima[name] is None or clearance < minima[name]:
                minima[name] = clearance
    return minima


def compass_yaw(compass_values: Sequence[float]) -> float:
    """Convert Webots compass north vector to ENU heading (CCW from world +X).

    This assumes world +Y is north and the robot's local +X is forward.
    """
    if len(compass_values) < 2:
        raise ValueError("compass must provide at least two components")
    north_x, north_y = float(compass_values[0]), float(compass_values[1])
    if not math.isfinite(north_x) or not math.isfinite(north_y):
        raise ValueError("invalid compass vector")
    if math.hypot(north_x, north_y) < 1e-9:
        raise ValueError("zero compass vector")
    return math.atan2(-north_x, north_y)
