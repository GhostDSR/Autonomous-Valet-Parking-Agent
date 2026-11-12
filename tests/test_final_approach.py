"""Tests for the exact final approach (planner/final_approach.py) and the pre-park pose."""

import math
import unittest

from planner.final_approach import approach_point, connect_to_goal
from planner.grid_map import GridMap
from planner.hybrid_astar import HybridAStarConfig, hybrid_astar
from planner.parking_lot import make_slots, pre_park_poses
from planner.vehicle import CollisionChecker, VehicleParams


def open_map(width=30, height=20):
    g = GridMap(width, height, 0.25)
    g.add_box(0, 0, width, 0.25)
    g.add_box(0, height - 0.25, width, height)
    g.add_box(0, 0, 0.25, height)
    g.add_box(width - 0.25, 0, width, height)
    return g


class TestFinalApproach(unittest.TestCase):
    def setUp(self):
        self.vehicle = VehicleParams()
        self.checker = CollisionChecker(open_map(), self.vehicle)

    def test_joins_exactly_and_is_drivable(self):
        goal = (20.0, 10.0, 0.0)
        tail = connect_to_goal((15.0, 10.4, math.radians(-6)), goal, self.vehicle, self.checker)
        self.assertIsNotNone(tail)
        self.assertEqual(tail[-1], goal)
        # Heading change between 0.25 m steps never exceeds what the car can turn.
        prev = (15.0, 10.4, math.radians(-6))
        for p in tail:
            step = math.hypot(p[0] - prev[0], p[1] - prev[1])
            self.assertLessEqual(abs(p[2] - prev[2]), step * self.vehicle.max_curvature * 1.1 + 1e-9)
            prev = p

    def test_refuses_goal_behind_or_too_sharp(self):
        goal = (20.0, 10.0, 0.0)
        self.assertIsNone(connect_to_goal((21.0, 10.0, 0.0), goal, self.vehicle, self.checker))
        # 2 m sideways in 2 m forward is far tighter than the 5.1 m turning radius.
        self.assertIsNone(connect_to_goal((18.0, 12.0, 0.0), goal, self.vehicle, self.checker))

    def test_approach_point(self):
        x, y, yaw = approach_point((10.0, 5.0, math.pi), 4.0)
        self.assertAlmostEqual(x, 14.0)
        self.assertAlmostEqual(y, 5.0)

    def test_hybrid_astar_ends_exactly_on_goal(self):
        goal = (22.0, 14.0, 0.0)
        config = HybridAStarConfig(final_approach_m=5.0)
        path, stats = hybrid_astar(open_map(), (4.0, 4.0, 0.0), goal, self.vehicle, config)
        self.assertTrue(stats["reached_goal"])
        self.assertEqual(path[-1], goal)
        self.assertEqual(len(path), len(stats["directions"]))


class TestPreParkPose(unittest.TestCase):
    def test_pre_park_pose_stands_off_from_the_bay(self):
        slots = make_slots()
        for sid in ("A3", "B3", "C3", "D3"):
            s = slots[sid]
            for x, y, yaw in pre_park_poses(s):
                # 4 m out from the bay entrance line, on the aisle side
                self.assertAlmostEqual(abs(y - s.y), 2.5 + 4.0)
                self.assertEqual((y - s.y) > 0, s.aisle_y > s.y)
                self.assertAlmostEqual(abs(x - s.x), 3.0)

if __name__ == "__main__":
    unittest.main()
