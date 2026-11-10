"""Unit tests for GridMap (planner/grid_map.py)."""

import unittest

from planner.grid_map import GridMap


class TestGridMap(unittest.TestCase):
    def test_world_grid_round_trip(self):
        g = GridMap(10, 5, resolution=0.5, origin=(-2.0, 1.0))
        self.assertEqual((g.cols, g.rows), (20, 10))
        col, row = g.world_to_grid(0.3, 2.2)
        self.assertEqual((col, row), (4, 2))
        x, y = g.grid_to_world(col, row)
        self.assertEqual(g.world_to_grid(x, y), (col, row))
        self.assertAlmostEqual(x, 0.25)  # centre of the cell
        self.assertAlmostEqual(y, 2.25)

    def test_outside_counts_as_occupied(self):
        g = GridMap(4, 4, 0.25)
        self.assertTrue(g.is_occupied_world(-0.1, 1.0))
        self.assertTrue(g.is_occupied_world(1.0, 4.1))
        self.assertFalse(g.is_occupied_world(1.0, 1.0))

    def test_box_marks_cells(self):
        g = GridMap(4, 4, 0.25)
        g.add_box(1.0, 1.0, 2.0, 1.5)
        self.assertTrue(g.is_occupied_world(1.5, 1.25))
        self.assertFalse(g.is_occupied_world(3.0, 3.0))
        self.assertFalse(g.is_occupied_world(1.5, 2.2))

    def test_rotated_rectangle(self):
        g = GridMap(10, 10, 0.25)
        g.add_rectangle(5, 5, length=4, width=1, yaw=1.5707963)  # long side along y
        self.assertTrue(g.is_occupied_world(5.0, 6.8))
        self.assertFalse(g.is_occupied_world(6.8, 5.0))

    def test_inflate(self):
        g = GridMap(5, 5, 0.25)
        g.add_box(2.5, 2.5, 2.75, 2.75)  # one cell
        big = g.inflate(0.5)
        self.assertTrue(big.is_occupied_world(3.1, 2.6))   # ~0.5 m away
        self.assertFalse(big.is_occupied_world(3.6, 2.6))  # ~1 m away
        self.assertFalse(g.is_occupied_world(3.1, 2.6), "inflate must not change the original")


if __name__ == "__main__":
    unittest.main()
