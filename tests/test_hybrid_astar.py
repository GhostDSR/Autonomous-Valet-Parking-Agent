"""Unit tests for the car model and Hybrid A* (planner/vehicle.py, planner/hybrid_astar.py)."""

import math
import unittest

from planner.grid_map import GridMap
from planner.hybrid_astar import HybridAStarConfig, hybrid_astar
from planner.vehicle import CollisionChecker, VehicleParams, move, wrap_angle


def open_map(width=30, height=20):
    g = GridMap(width, height, 0.25)
    g.add_box(0, 0, width, 0.25)
    g.add_box(0, height - 0.25, width, height)
    g.add_box(0, 0, 0.25, height)
    g.add_box(width - 0.25, 0, width, height)
    return g


class TestVehicleModel(unittest.TestCase):
    def test_straight(self):
        x, y, yaw = move(0, 0, 0, 2.0, 0.0, 2.8)
        self.assertAlmostEqual(x, 2.0)
        self.assertAlmostEqual(y, 0.0)

    def test_full_circle_returns_to_start(self):
        v = VehicleParams()
        r = v.min_turn_radius
        x, y, yaw = move(1, 2, 0.3, 2 * math.pi * r, v.max_steer, v.wheelbase)
        self.assertAlmostEqual(x, 1, places=6)
        self.assertAlmostEqual(y, 2, places=6)
        self.assertAlmostEqual(wrap_angle(yaw - 0.3), 0, places=6)

    def test_left_turn_goes_left_and_reverse_undoes_forward(self):
        x, y, yaw = move(0, 0, 0, 1.0, 0.4, 2.8)
        self.assertGreater(y, 0)
        self.assertGreater(yaw, 0)
        bx, by, byaw = move(x, y, yaw, -1.0, 0.4, 2.8)
        self.assertAlmostEqual(bx, 0, places=9)
        self.assertAlmostEqual(by, 0, places=9)
        self.assertAlmostEqual(byaw, 0, places=9)

    def test_collision_checker(self):
        g = open_map()
        ck = CollisionChecker(g, VehicleParams())
        self.assertFalse(ck.pose_collides(10, 10, 0))
        self.assertTrue(ck.pose_collides(27.5, 10, 0))   # nose in the right wall
        self.assertFalse(ck.pose_collides(27.5, 10, math.pi))  # same spot, facing away


class TestHybridAStar(unittest.TestCase):
    def setUp(self):
        self.vehicle = VehicleParams()
        self.config = HybridAStarConfig()

    def check_path(self, grid, path, stats, goal):
        ck = CollisionChecker(grid, self.vehicle)
        self.assertTrue(stats["reached_goal"])
        self.assertEqual(len(path), len(stats["directions"]))
        end = path[-1]
        self.assertLessEqual(math.hypot(end[0] - goal[0], end[1] - goal[1]),
                             self.config.goal_xy_tolerance + 1e-9)
        self.assertLessEqual(abs(wrap_angle(end[2] - goal[2])),
                             math.radians(self.config.goal_yaw_tolerance_deg) + 1e-9)
        for a, b in zip(path, path[1:]):
            self.assertFalse(ck.pose_collides(*b))
            # Drivable: consecutive poses are `sub_step` metres of arc apart, so
            # the heading change between them can be at most sub_step / R_min.
            # (Not the straight-line gap: a chord is slightly shorter than its arc.)
            self.assertLessEqual(abs(wrap_angle(b[2] - a[2])),
                                 self.config.sub_step * self.vehicle.max_curvature + 1e-9)

    def test_reaches_goal_with_turn(self):
        g = open_map()
        start, goal = (4, 4, 0.0), (20, 14, math.pi / 2)
        path, stats = hybrid_astar(g, start, goal, self.vehicle, self.config)
        self.check_path(g, path, stats, goal)
        self.assertEqual(path[0], start)

    def test_goes_around_wall(self):
        g = open_map()
        g.add_box(14, 0, 15, 14)  # wall with a gap at the top
        start, goal = (4, 5, 0.0), (24, 5, 0.0)
        path, stats = hybrid_astar(g, start, goal, self.vehicle, self.config)
        self.check_path(g, path, stats, goal)
        self.assertGreater(max(p[1] for p in path), 14)  # used the gap

    def test_unreachable_goal(self):
        g = open_map()
        g.add_box(14, 0, 15, 20)  # wall right across the map
        path, stats = hybrid_astar(g, (4, 10, 0.0), (24, 10, 0.0), self.vehicle, self.config)
        self.assertEqual(path, [])
        self.assertFalse(stats["reached_goal"])

    def test_colliding_start_raises(self):
        g = open_map()
        with self.assertRaises(ValueError):
            hybrid_astar(g, (0.5, 10, 0.0), (20, 10, 0.0), self.vehicle, self.config)


if __name__ == "__main__":
    unittest.main()
