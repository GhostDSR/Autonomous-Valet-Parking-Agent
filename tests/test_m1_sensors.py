import math
import unittest

from controllers.valet_controller.m1_sensors import (
    body_clearances_from_scan,
    classify_slots_from_scan,
    compass_yaw,
    lidar_points_world,
    webots_ranges_to_ccw,
)
from planner.parking_lot import make_slots


class M1SensorTests(unittest.TestCase):
    def test_compass_heading(self):
        # ENU world, north = +Y. The compass reports north in the robot frame:
        # facing east (yaw 0) north is on the left (+y); facing north
        # (yaw +90 deg) north is straight ahead (+x).
        self.assertAlmostEqual(compass_yaw((0.0, 1.0, 0.0)), 0.0)
        self.assertAlmostEqual(compass_yaw((1.0, 0.0, 0.0)), math.pi / 2)
        self.assertAlmostEqual(compass_yaw((-1.0, 0.0, 0.0)), -math.pi / 2)
        self.assertAlmostEqual(abs(compass_yaw((0.0, -1.0, 0.0))), math.pi)

    def test_compass_round_trip(self):
        for yaw in (-2.5, -1.0, 0.0, 0.7, 2.0, 3.0):
            reading = (math.sin(yaw), math.cos(yaw), 0.0)
            self.assertAlmostEqual(compass_yaw(reading), yaw)

    def test_lidar_world_transform(self):
        points = lidar_points_world([5.0, 5.0, 5.0], (1.0, 2.0, 0.0),
                                    field_of_view=math.pi, max_range=10.0)
        self.assertEqual(len(points), 3)
        self.assertAlmostEqual(points[1][0], 6.0)
        self.assertAlmostEqual(points[1][1], 2.0)

    def test_invalid_ranges_are_ignored(self):
        points = lidar_points_world([float("inf"), 0.0, 20.0], (0, 0, 0),
                                    field_of_view=math.pi, max_range=20.0)
        self.assertEqual(points, [])

    @staticmethod
    def _room_scan(n=721, front=10.0, rear=10.0, left=10.0, right=10.0):
        """Range image (CCW, -pi..pi) of a rectangular room around the axle."""
        ranges = []
        for i in range(n):
            a = -math.pi + i * 2 * math.pi / (n - 1)
            c, s_ = math.cos(a), math.sin(a)
            hits = []
            if c > 1e-9:
                hits.append(front / c)
            if c < -1e-9:
                hits.append(-rear / c)
            if s_ > 1e-9:
                hits.append(left / s_)
            if s_ < -1e-9:
                hits.append(-right / s_)
            ranges.append(min(hits))
        return ranges

    def test_body_clearance_subtracts_vehicle_footprint(self):
        # Flat walls 10 m from the rear axle on every side. Footprint:
        # 3.6 m ahead of the axle, 1.0 m behind it, 0.95 m to each side.
        clearances = body_clearances_from_scan(self._room_scan(), (0, 0, 0))
        self.assertAlmostEqual(clearances["front"], 10.0 - 3.6)
        self.assertAlmostEqual(clearances["rear"], 10.0 - 1.0)
        self.assertAlmostEqual(clearances["left"], 10.0 - 0.95)
        self.assertAlmostEqual(clearances["right"], 10.0 - 0.95)

    def test_body_clearance_tracks_each_side_independently(self):
        clearances = body_clearances_from_scan(
            self._room_scan(front=5.0, rear=4.0, left=3.0, right=6.0), (0, 0, 0))
        self.assertAlmostEqual(clearances["front"], 5.0 - 3.6)
        self.assertAlmostEqual(clearances["rear"], 4.0 - 1.0)
        self.assertAlmostEqual(clearances["left"], 3.0 - 0.95)
        self.assertAlmostEqual(clearances["right"], 6.0 - 0.95)

    def test_oblique_return_uses_distance_to_footprint(self):
        # One return at 20 deg left of straight ahead, 10 m away: the point is
        # (9.397, 3.420). It is 5.797 m beyond the bumper and 2.470 m outside
        # the side of the footprint.
        ranges = [float("inf")] * 721
        ranges[360 + 40] = 10.0  # 721 samples over 2*pi -> 0.5 deg each
        clearances = body_clearances_from_scan(ranges, (0, 0, 0))
        expected = math.hypot(10.0 * math.cos(math.radians(20)) - 3.6,
                              10.0 * math.sin(math.radians(20)) - 0.95)
        self.assertAlmostEqual(clearances["front"], expected)
        self.assertIsNone(clearances["rear"])
        self.assertIsNone(clearances["left"])
        self.assertIsNone(clearances["right"])

    def test_side_obstacle_beside_the_body_is_close(self):
        # An object 2 m to the left of the axle is 1.05 m from the body side.
        ranges = [float("inf")] * 721
        ranges[540] = 2.0
        self.assertAlmostEqual(
            body_clearances_from_scan(ranges, (0, 0, 0))["left"], 2.0 - 0.95)

    def test_missing_sector_is_none_not_large(self):
        clearances = body_clearances_from_scan([float("inf")] * 721, (0, 0, 0))
        self.assertEqual(set(clearances.values()), {None})

    def test_webots_range_image_is_reordered_counter_clockwise(self):
        # Webots sample 0 is the sensor's left (+FOV/2); the last is its right.
        raw = [1.0, 2.0, 3.0]
        self.assertEqual(webots_ranges_to_ccw(raw), [3.0, 2.0, 1.0])
        # A return on the vehicle's left (+y) is raw sample 0 of a 180-degree
        # image; after reordering it must project onto +y in the world.
        raw = [4.0, float("inf"), float("inf")]
        points = lidar_points_world(webots_ranges_to_ccw(raw), (0.0, 0.0, 0.0),
                                    field_of_view=math.pi, max_range=10.0)
        self.assertEqual(len(points), 1)
        self.assertAlmostEqual(points[0][0], 0.0, places=6)
        self.assertAlmostEqual(points[0][1], 4.0)

    def test_slot_classification_keeps_occluded_bays_unknown(self):
        slots = {"A0": make_slots()["A0"]}
        # No scan evidence means unknown, not falsely free.
        result = classify_slots_from_scan([], (2.0, 8.5, 0.0), slots=slots)
        self.assertEqual(result["A0"], "unknown")

    def test_empty_slot_can_be_visible_and_free(self):
        slots = {"A0": make_slots()["A0"]}
        # Rays from the entrance along +X do not cross A0; test a ray from
        # directly below the bay through its centre, returning beyond it.
        # Use a dense 360-degree scan; only the ray directed +Y has a return.
        ranges = [float("inf")] * 721
        ranges[540] = 30.0
        result = classify_slots_from_scan(
            ranges, (9.25, -10.0, 0.0), slots=slots,
            field_of_view=2 * math.pi, max_range=40.0,
        )
        self.assertEqual(result["A0"], "free")


if __name__ == "__main__":
    unittest.main()
