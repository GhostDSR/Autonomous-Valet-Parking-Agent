"""Tests for the integrated valet agent (agent/)."""

import math
import unittest

from agent.sim_world import SimWorld, ray_box_distances
from agent.valet_agent import body_clearances
from planner.parking_lot import make_slots


class TestSensing(unittest.TestCase):
    def test_ray_box_distances(self):
        box = [(10.0, 0.0, 2.0, 2.0, 0.0)]          # square from x = 9 to 11
        d = ray_box_distances(0.0, 0.0, [0.0, math.pi, math.pi / 2], box, 40.0)
        self.assertAlmostEqual(d[0], 9.0)
        self.assertTrue(math.isinf(d[1]))           # facing away
        self.assertTrue(math.isinf(d[2]))           # passes beside it

    def test_lidar_sees_walls(self):
        world = SimWorld(set())
        ranges = world.lidar_scan((20.0, 8.5, 0.0))
        n = len(ranges)
        self.assertAlmostEqual(ranges[n // 2], 40.0 - 0.25 - 20.0)       # straight ahead: right wall
        self.assertAlmostEqual(ranges[3 * n // 4], 34.0 - 0.25 - 8.5)    # to the left: top wall

    def test_body_clearances_exact(self):
        # Obstacles 10 m from the rear axle all round: the nearest are off the
        # front corners, hypot(3.6, 0.95) = 3.72 m from the axle.
        gaps = body_clearances([10.0] * 721, 2 * math.pi, 3.6, 1.0, 0.95)
        self.assertAlmostEqual(gaps["front"], 10.0 - math.hypot(3.6, 0.95), places=2)
        self.assertAlmostEqual(gaps["rear"], 10.0 - math.hypot(1.0, 0.95), places=2)

    def test_body_clearances_parked_between_cars(self):
        # Parked with a car 0.6 m away on each side: the side gaps are 0.6 m,
        # and those cars do not count as being in front or behind.
        world = SimWorld({"B2", "B4"})
        s = make_slots()["B3"]
        pose = (s.x, s.y + 1.3, s.yaw)                      # rear axle of a car centred in B3
        gaps = body_clearances(world.lidar_scan(pose), 2 * math.pi, 3.6, 1.0, 0.95)
        self.assertAlmostEqual(gaps["left"], 0.6, places=2)
        self.assertAlmostEqual(gaps["right"], 0.6, places=2)
        self.assertGreater(gaps["front"], 2.0)


class TestWholeAgent(unittest.TestCase):
    def test_parks_from_the_entrance(self):
        from agent.run_offline import run
        res = run("normal")
        self.assertEqual(res["phase"], "DONE", res["reason"])
        self.assertGreater(res["min_gap_m"], 0.05)         # never touched anything
        self.assertLess(res["centre_error_cm"], 15)
        self.assertLess(res["heading_error_deg"], 3)


if __name__ == "__main__":
    unittest.main()
