"""Integration tests: parking lot + route planner + metrics."""

import math
import unittest

from planner import (ENTRANCE_POSE, METHODS, CollisionChecker, VehicleParams,
                     build_parking_lot, free_slots, plan_route, plan_to_slot)
from planner.metrics import compute_metrics, curvature_profile, path_length


class TestParkingLot(unittest.TestCase):
    def test_slots_and_occupancy(self):
        grid, slots = build_parking_lot(occupied={"A0"})
        self.assertEqual(len(slots), 40)
        self.assertTrue(grid.is_occupied_world(slots["A0"].x, slots["A0"].y))
        self.assertFalse(grid.is_occupied_world(slots["A1"].x, slots["A1"].y))
        self.assertIn("A1", free_slots(slots, {"A0"}))

    def test_unknown_slot_rejected(self):
        with self.assertRaises(ValueError):
            build_parking_lot(occupied={"Z9"})

    def test_entrance_pose_is_free(self):
        grid, _ = build_parking_lot()
        self.assertFalse(CollisionChecker(grid, VehicleParams()).pose_collides(*ENTRANCE_POSE))


class TestRoutePlanner(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.grid, cls.slots = build_parking_lot()
        cls.vehicle = VehicleParams()
        cls.checker = CollisionChecker(cls.grid, cls.vehicle)

    def test_all_methods_return_poses(self):
        for method in METHODS:
            path, stats, goal = plan_to_slot(self.grid, ENTRANCE_POSE, self.slots["D6"], method,
                                             self.vehicle, checker=self.checker)
            self.assertTrue(path, method)
            self.assertEqual(len(path[0]), 3)
            self.assertEqual(len(stats["directions"]), len(path))
            self.assertLess(math.hypot(path[-1][0] - goal[0], path[-1][1] - goal[1]), 0.6)

    def test_hybrid_path_is_drivable_and_collision_free(self):
        path, stats, goal = plan_to_slot(self.grid, ENTRANCE_POSE, self.slots["C9"],
                                         "hybrid_astar", self.vehicle, checker=self.checker)
        m = compute_metrics(path, self.vehicle, self.checker, goal, stats["directions"])
        self.assertEqual(m["footprint_hits"], 0)
        self.assertEqual(m["over_limit_m"], 0.0)
        self.assertLess(m["goal_yaw_error_deg"], 10.0 + 1e-6)

    def test_plan_route_rejects_unknown_method(self):
        with self.assertRaises(ValueError):
            plan_route(self.grid, ENTRANCE_POSE, (20, 8.5, 0), method="dijkstra")


class TestMetrics(unittest.TestCase):
    def test_length(self):
        self.assertAlmostEqual(path_length([(0, 0), (3, 4), (3, 5)]), 6.0)

    def test_circle_curvature(self):
        r = 4.0
        pts = [(r * math.cos(t / 20), r * math.sin(t / 20)) for t in range(30)]
        _, kappas = curvature_profile(pts)
        for k in kappas:
            self.assertAlmostEqual(k, 1 / r, places=6)

    def test_straight_line_has_zero_curvature(self):
        _, kappas = curvature_profile([(0, 0), (1, 1), (2, 2), (3, 3)])
        self.assertEqual(kappas, [0.0, 0.0])


if __name__ == "__main__":
    unittest.main()
