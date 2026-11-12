"""Static checks of worlds/parking_lot.wbt (CI cannot run Webots itself).

These guard the things that broke silently before: missing actuator devices,
wrong device names, a floor that did not cover the lot, a lidar rotated out of
the horizontal plane, and a rear distance sensor that faced forward.
"""
import math
import os
import re
import unittest

from planner.parking_lot import (
    BAY_DEPTH, BAY_WIDTH, DEFAULT_OCCUPIED, PARKED_CAR_LENGTH, PARKED_CAR_WIDTH,
    ROWS, WALL_THICKNESS, make_slots,
)

WORLD = os.path.join(os.path.dirname(__file__), "..", "worlds", "parking_lot.wbt")

ACTUATORS = (
    "front_left_steer", "front_right_steer",
    "front_left_wheel_motor", "front_right_wheel_motor",
    "rear_left_wheel_motor", "rear_right_wheel_motor",
)
SENSORS = ("gps", "compass", "imu", "lidar",
           "ds_front", "ds_rear", "ds_left", "ds_right")


def _read():
    with open(WORLD) as handle:
        return handle.read()


def _robot_text():
    return _read()[_read().index("Robot {"):]


def _block(text, header):
    """Text of the brace-balanced block that starts at ``header``."""
    start = text.index(header)
    depth, i = 0, text.index("{", start)
    first = i
    while True:
        if text[i] == "{":
            depth += 1
        elif text[i] == "}":
            depth -= 1
            if depth == 0:
                return text[first:i + 1]
        i += 1


class WorldFileTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.world = _read()
        cls.robot = _robot_text()

    def test_braces_and_brackets_balance(self):
        self.assertEqual(self.world.count("{"), self.world.count("}"))
        self.assertEqual(self.world.count("["), self.world.count("]"))

    def test_every_proto_node_is_declared_with_externproto(self):
        # Without EXTERNPROTO, Webots R2023b+ skips the node ("Skipped unknown
        # 'Floor' node"), the lot has no ground and the car falls through it.
        for proto in ("Floor", "TexturedBackground", "TexturedBackgroundLight"):
            self.assertRegex(self.world, r"(?m)^%s \{" % proto)
            self.assertRegex(self.world, r'(?m)^EXTERNPROTO "[^"\n]*/%s\.proto"$' % proto)

    def test_device_names_are_unique_and_present(self):
        names = re.findall(r'name "([^"]+)"', self.world)
        self.assertEqual(len(names), len(set(names)))
        for name in ACTUATORS + SENSORS:
            self.assertIn(name, names)

    def test_every_actuator_is_a_rotational_motor(self):
        for name in ACTUATORS:
            self.assertRegex(self.robot, r'RotationalMotor\s*\{\s*name "%s"' % name)

    def test_four_wheels_and_two_steering_joints(self):
        self.assertEqual(len(re.findall(r"_wheel_motor\"", self.robot)), 4)
        self.assertEqual(len(re.findall(r'_steer"', self.robot)), 2)

    def test_steering_range_covers_inner_wheel_at_max_bicycle_angle(self):
        limit = float(re.search(r"front_left_steer\"\s+minPosition -([\d.]+)", self.robot).group(1))
        wheelbase, track, delta = 2.8, 1.6, 0.5
        rho = wheelbase / math.tan(delta)
        inner = math.atan(wheelbase / (rho - track / 2))
        self.assertGreater(limit, inner)

    def test_front_axle_is_one_wheelbase_ahead_of_the_rear_axle(self):
        anchors = re.findall(r"axis 0 0 1 anchor ([\d.-]+) ([\d.-]+) 0", self.robot)
        self.assertEqual(len(anchors), 2)
        self.assertEqual({a[0] for a in anchors}, {"2.8"})
        self.assertEqual({a[1] for a in anchors}, {"0.8", "-0.8"})
        rear = re.findall(r"axis 0 1 0 anchor 0 ([\d.-]+) 0", self.robot)
        self.assertEqual(set(rear), {"0.8", "-0.8"})

    def test_robot_origin_height_equals_wheel_radius(self):
        z = float(re.search(r"translation 2 8\.5 ([\d.]+)", self.robot).group(1))
        radius = float(re.search(r"radius ([\d.]+) \}", _block(self.robot, "DEF WHEEL")).group(1))
        self.assertAlmostEqual(z, radius)

    def test_floor_covers_the_whole_lot(self):
        match = re.search(r"Floor \{ translation ([\d.]+) ([\d.]+) 0 size ([\d.]+) ([\d.]+) \}", self.world)
        self.assertIsNotNone(match, "Floor must be translated to the lot centre")
        cx, cy, sx, sy = map(float, match.groups())
        self.assertLessEqual(cx - sx / 2, 0.0)
        self.assertGreaterEqual(cx + sx / 2, 40.0)
        self.assertLessEqual(cy - sy / 2, 0.0)
        self.assertGreaterEqual(cy + sy / 2, 34.0)

    def test_lidar_scans_the_horizontal_plane(self):
        # Webots devices are FLU: a horizontal scan needs no rotation.
        self.assertNotIn("rotation", _block(self.robot, "Lidar {"))

    def test_rear_distance_sensor_faces_backwards(self):
        line = next(l for l in self.robot.splitlines() if 'name "ds_rear"' in l)
        angle = float(re.search(r"rotation 0 0 1 ([\d.]+)", line).group(1))
        self.assertAlmostEqual(angle, math.pi, places=4)

    def test_lidar_plane_is_above_the_chassis_and_below_the_cabin(self):
        z = float(re.search(r"Lidar \{.*?translation 0 0 ([\d.]+)", self.robot, re.S).group(1))
        chassis_top = 0.05 + 0.3 / 2
        cabin_bottom = 0.54 - 0.32 / 2
        self.assertGreater(z, chassis_top)
        self.assertLess(z, cabin_bottom)

    def test_lidar_plane_hits_walls_and_parked_cars(self):
        z_world = 0.3 + float(re.search(r"Lidar \{.*?translation 0 0 ([\d.]+)", self.robot, re.S).group(1))
        self.assertLess(z_world, 1.0)   # walls are 1.0 m tall, parked cars 1.3 m


def _solids():
    """{name: (x, y, yaw, size_x, size_y)} for every Solid with a Box bounding object."""
    out = {}
    pattern = re.compile(
        r'Solid \{\n  translation (\S+) (\S+) \S+\n(?:  rotation 0 0 1 (\S+)\n)?'
        r'  name "([^"]+)".*?boundingObject Box \{ size (\S+) (\S+) \S+ \}', re.S)
    for x, y, rot, name, sx, sy in pattern.findall(_read()):
        out[name] = (float(x), float(y), float(rot or 0.0), float(sx), float(sy))
    return out


class WorldMatchesPlannerTests(unittest.TestCase):
    """The world and planner/parking_lot.py must describe the same lot."""

    @classmethod
    def setUpClass(cls):
        cls.solids = _solids()
        cls.slots = make_slots()

    def test_bay_lines_sit_on_the_planner_rows(self):
        for name, y_bottom, _yaw, _aisle in ROWS:
            for i in range(11):                      # 10 bays -> 11 dividing lines
                x, y, _, _, length = self.solids["line_%s_%d" % (name, i)]
                self.assertAlmostEqual(x, 8.0 + i * BAY_WIDTH)
                self.assertAlmostEqual(y, y_bottom + BAY_DEPTH / 2, msg="line_%s_%d" % (name, i))
                self.assertAlmostEqual(length, BAY_DEPTH)

    def test_row_lines_sit_on_the_planner_row_edges(self):
        for name, y_bottom, _yaw, _aisle in ROWS:
            self.assertAlmostEqual(self.solids["row_%s_backline" % name][1], y_bottom)

    def test_parked_cars_are_exactly_the_planner_occupied_bays(self):
        cars = {n[len("parked_car_"):] for n in self.solids if n.startswith("parked_car_")}
        self.assertEqual(cars, set(DEFAULT_OCCUPIED))

    def test_parked_cars_are_centred_in_their_bays(self):
        for sid in DEFAULT_OCCUPIED:
            x, y, _, _, _ = self.solids["parked_car_" + sid]
            self.assertAlmostEqual(x, self.slots[sid].x, msg=sid)
            self.assertAlmostEqual(y, self.slots[sid].y, msg=sid)

    def test_parked_cars_lie_along_their_bays(self):
        # A car is PARKED_CAR_LENGTH long in the direction of its bay's depth (world y)
        # and PARKED_CAR_WIDTH across it; unrotated boxes would be 4.5 m wide in a 2.5 m bay.
        for sid in DEFAULT_OCCUPIED:
            _, _, yaw, sx, sy = self.solids["parked_car_" + sid]
            across_x = abs(sx * math.cos(yaw)) + abs(sy * math.sin(yaw))
            along_y = abs(sx * math.sin(yaw)) + abs(sy * math.cos(yaw))
            self.assertAlmostEqual(across_x, PARKED_CAR_WIDTH, places=3, msg=sid)
            self.assertAlmostEqual(along_y, PARKED_CAR_LENGTH, places=3, msg=sid)

    def test_rows_a_and_d_stand_clear_of_the_outer_wall(self):
        # Rear of a parked car, and the row's back line, must be off the wall's inner face.
        for sid in ("A0", "D0"):
            slot = self.slots[sid]
            wall_face = WALL_THICKNESS if sid[0] == "A" else 34.0 - WALL_THICKNESS
            sign = -1.0 if sid[0] == "A" else 1.0
            car_rear = slot.y + sign * PARKED_CAR_LENGTH / 2
            self.assertGreaterEqual(abs(car_rear - wall_face), 0.4)
        self.assertAlmostEqual(self.solids["row_A_backline"][1], 0.5)


if __name__ == "__main__":
    unittest.main()
