"""Unit tests for grid A* (planner/astar.py)."""

import math
import unittest

from planner.astar import astar, make_test_lot, octile, plan_path


class TestAStar(unittest.TestCase):
    def test_straight_line_cost(self):
        path, stats = astar([[0] * 5], (0, 0), (4, 0))
        self.assertEqual(path, [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0)])
        self.assertAlmostEqual(stats["cost"], 4.0)

    def test_diagonal_is_used_when_allowed(self):
        grid = [[0] * 4 for _ in range(4)]
        _, stats = astar(grid, (0, 0), (3, 3))
        self.assertAlmostEqual(stats["cost"], 3 * math.sqrt(2))
        _, stats4 = astar(grid, (0, 0), (3, 3), allow_diagonal=False)
        self.assertAlmostEqual(stats4["cost"], 6.0)

    def test_path_is_valid_on_test_lot(self):
        grid, start, goal = make_test_lot()
        path = plan_path(grid, start, goal)
        self.assertEqual(path[0], start)
        self.assertEqual(path[-1], goal)
        for x, y in path:
            self.assertEqual(grid[y][x], 0, "path goes through an obstacle")
        for a, b in zip(path, path[1:]):
            self.assertEqual(max(abs(a[0] - b[0]), abs(a[1] - b[1])), 1, "path skips a cell")

    def test_goes_around_a_wall(self):
        grid = [[0, 0, 0],
                [1, 1, 0],
                [0, 0, 0]]
        path = plan_path(grid, (0, 0), (0, 2))
        self.assertIn((2, 1), path)  # only gap in the wall

    def test_unreachable_goal_returns_empty(self):
        box = [[0, 0, 0, 0, 0],
               [0, 1, 1, 1, 0],
               [0, 1, 0, 1, 0],
               [0, 1, 1, 1, 0],
               [0, 0, 0, 0, 0]]
        self.assertEqual(plan_path(box, (0, 0), (2, 2)), [])

    def test_no_corner_cutting(self):
        self.assertEqual(plan_path([[0, 1], [1, 0]], (0, 0), (1, 1)), [])

    def test_bad_start_or_goal_raises(self):
        grid = [[0, 1], [0, 0]]
        with self.assertRaises(ValueError):
            plan_path(grid, (1, 0), (0, 1))   # start in obstacle
        with self.assertRaises(ValueError):
            plan_path(grid, (0, 0), (5, 5))   # goal outside grid

    def test_octile_never_overestimates(self):
        # On an empty grid the octile distance equals the true A* cost.
        grid = [[0] * 6 for _ in range(6)]
        for goal in [(5, 0), (5, 5), (2, 4)]:
            _, stats = astar(grid, (0, 0), goal)
            self.assertAlmostEqual(octile((0, 0), goal), stats["cost"])


if __name__ == "__main__":
    unittest.main()
