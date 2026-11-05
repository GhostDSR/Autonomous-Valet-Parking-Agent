"""
Car model for the planner: size, steering limits, motion and collision checks.

Part of M2 - Route Planner.

Kinematic bicycle model
-----------------------
An Ackermann car is modelled as a "bicycle": one rear wheel and one
steerable front wheel, a distance L (the wheelbase) apart. The reference
point is the centre of the rear axle. With steering angle delta the car
drives on a circle of radius

    R = L / tan(delta)

so the tightest circle it can drive is R_min = L / tan(delta_max).
This is the constraint plain A* ignores and Hybrid A* respects.

Collision model
---------------
The car's rectangle is covered by a few equal circles placed along its
centre line. Instead of checking a rotated rectangle against the grid, we
inflate the obstacles by the circle radius once, and then only need to look
up each circle *centre* in the inflated grid (a handful of array lookups).
This is slightly conservative: the circles stick out a little beyond the
real car body.

All default numbers below are PLACEHOLDERS for a mid-size car. They must be
replaced by the values of the actual Webots car model M1 uses.
"""

import math
from dataclasses import dataclass


def wrap_angle(angle):
    """Wrap an angle to the range [-pi, pi)."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


@dataclass
class VehicleParams:
    wheelbase: float = 2.8       # m, rear axle to front axle
    max_steer: float = 0.5       # rad (~28.6 deg), max front-wheel angle
    length: float = 4.6          # m, bumper to bumper
    width: float = 1.9           # m
    rear_overhang: float = 1.0   # m, rear bumper to rear axle
    n_circles: int = 3           # circles used to cover the body
    safety_margin: float = 0.1   # m, extra clearance added to each circle

    @property
    def min_turn_radius(self):
        return self.wheelbase / math.tan(self.max_steer)

    @property
    def max_curvature(self):
        return 1.0 / self.min_turn_radius

    def collision_circles(self):
        """
        Return (offsets, radius): circle centres as distances ahead of the
        rear axle (negative = behind it), and the common circle radius.
        """
        segment = self.length / self.n_circles
        offsets = [-self.rear_overhang + segment * (i + 0.5) for i in range(self.n_circles)]
        radius = math.hypot(segment / 2, self.width / 2) + self.safety_margin
        return offsets, radius


def move(x, y, yaw, distance, steer, wheelbase):
    """
    Drive the bicycle model `distance` metres (negative = reverse) with a
    fixed steering angle, and return the new rear-axle pose (x, y, yaw).

    With fixed steering the rear axle follows an exact circular arc (or a
    straight line when steer == 0), so we use the closed-form arc instead of
    many small Euler steps.
    """
    if abs(steer) < 1e-9:
        return x + distance * math.cos(yaw), y + distance * math.sin(yaw), yaw

    radius = wheelbase / math.tan(steer)       # signed: + turns left, - turns right
    new_yaw = yaw + distance / radius          # heading change = arc length / radius
    new_x = x + radius * (math.sin(new_yaw) - math.sin(yaw))
    new_y = y + radius * (math.cos(yaw) - math.cos(new_yaw))
    return new_x, new_y, wrap_angle(new_yaw)


class CollisionChecker:
    """
    Fast collision checks for a car on a GridMap.

    Builds the inflated map once (obstacles grown by the circle radius) and
    then answers "does the car at pose (x, y, yaw) hit anything?".
    """

    def __init__(self, grid_map, vehicle):
        self.vehicle = vehicle
        self.offsets, self.radius = vehicle.collision_circles()
        self.inflated = grid_map.inflate(self.radius)

        # Cache plain Python values: these lookups run millions of times.
        self._cells = self.inflated.as_lists()
        self._res = grid_map.resolution
        self._ox, self._oy = grid_map.origin_x, grid_map.origin_y
        self._rows, self._cols = grid_map.rows, grid_map.cols

    def point_collides(self, x, y):
        """True if a disc of radius `self.radius` centred at (x, y) hits an obstacle."""
        col = int(math.floor((x - self._ox) / self._res))
        row = int(math.floor((y - self._oy) / self._res))
        if col < 0 or row < 0 or col >= self._cols or row >= self._rows:
            return True
        return self._cells[row][col] != 0

    def pose_collides(self, x, y, yaw):
        """True if the car body (rear axle at (x, y), heading yaw) hits an obstacle."""
        c, s = math.cos(yaw), math.sin(yaw)
        for offset in self.offsets:
            if self.point_collides(x + offset * c, y + offset * s):
                return True
        return False
