import math
import unittest

from controllers.valet_controller.m1_sensors import (
    body_clearances_from_scan,
    classify_slots_from_scan,
    compass_yaw,
    lidar_points_world,
)
from planner.parking_lot import make_slots


class M1SensorTests(unittest.TestCase):
    def test_compass_heading(self):
        self.assertAlmostEqual(compass_yaw((0.0, 1.0, 0.0)), 0.0)
        self.assertAlmostEqual(compass_yaw((-1.0, 0.0, 0.0)), math.pi / 2)

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

    def test_body_clearance_subtracts_vehicle_footprint(self):
        ranges = [10.0] * 721
        clearances = body_clearances_from_scan(ranges, (0, 0, 0))
        self.assertAlmostEqual(clearances["front"], 10.0 - 3.6)
        self.assertAlmostEqual(clearances["rear"], 10.0 - 1.0)
        self.assertAlmostEqual(clearances["left"], 10.0 - 0.95, places=1)
        self.assertAlmostEqual(clearances["right"], 10.0 - 0.95, places=1)

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
