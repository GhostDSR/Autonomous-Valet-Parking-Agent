"""Unit tests for slot choice and replanning (planner/replanning.py)."""

import unittest

from planner.parking_lot import DEFAULT_OCCUPIED, ENTRANCE_POSE
from planner.replanning import (SlotChoice, build_map, check_route, choose_slot,
                                first_blocked_index, nearest_index, rank_slots, replan)
from planner.vehicle import CollisionChecker, VehicleParams


def free_ids(slots, occupied):
    return [sid for sid in slots if sid not in occupied]


class TestRouteChecks(unittest.TestCase):
    def test_nearest_index_only_searches_forward(self):
        path = [(0, 0, 0), (1, 0, 0), (2, 0, 0), (1, 0, 0)]  # passes x=1 twice
        self.assertEqual(nearest_index(path, (1.1, 0)), 1)
        self.assertEqual(nearest_index(path, (1.1, 0), start_index=2), 3)

    def test_first_blocked_index(self):
        clear = build_map(DEFAULT_OCCUPIED, [])[0]
        blocked = build_map(DEFAULT_OCCUPIED, [(10.0, 8.5, 0.6, 0.6, 0.0)])[0]
        path = [(x * 0.5, 8.5, 0.0) for x in range(10, 30)]   # along aisle 1, x = 5 .. 14.5
        v = VehicleParams()
        self.assertIsNone(first_blocked_index(path, CollisionChecker(clear, v)))
        i = first_blocked_index(path, CollisionChecker(blocked, v))
        self.assertIsNotNone(i)
        # Already past the obstacle: the rest of the route is clear.
        self.assertIsNone(first_blocked_index(path, CollisionChecker(blocked, v), start_index=len(path) - 1))

    def test_check_route_reasons(self):
        grid, slots = build_map(DEFAULT_OCCUPIED)
        checker = CollisionChecker(grid, VehicleParams())
        choice = SlotChoice("A2", [(5.0, 8.5, 0.0), (6.0, 8.5, 0.0)], {}, (6.0, 8.5, 0.0))
        self.assertIsNone(check_route(choice, 0, DEFAULT_OCCUPIED, checker))
        self.assertIn("occupied", check_route(choice, 0, DEFAULT_OCCUPIED | {"A2"}, checker))


class TestSlotChoice(unittest.TestCase):
    def test_rank_prefers_nearer_slots(self):
        grid, slots = build_map(DEFAULT_OCCUPIED)
        checker = CollisionChecker(grid, VehicleParams())
        ranked = rank_slots(grid, ENTRANCE_POSE, slots, free_ids(slots, DEFAULT_OCCUPIED), checker)
        lengths = [d for d, _ in ranked]
        self.assertEqual(lengths, sorted(lengths))
        self.assertEqual(len(ranked), len(free_ids(slots, DEFAULT_OCCUPIED)))
        # The entrance opens onto aisle 1, so the best slot is in row A or B,
        # and the bay nearest the entrance on aisle 1 (B0) beats the far end (B8).
        self.assertIn(ranked[0][1][0], "AB")
        order = [sid for _, sid in ranked]
        self.assertLess(order.index("B0"), order.index("B8"))

    def test_choose_slot_skips_excluded(self):
        grid, slots = build_map(DEFAULT_OCCUPIED)
        free = free_ids(slots, DEFAULT_OCCUPIED)
        first = choose_slot(grid, ENTRANCE_POSE, slots, free, method="astar")
        second = choose_slot(grid, ENTRANCE_POSE, slots, free, method="astar", exclude={first.slot_id})
        self.assertIn(first.slot_id, free)
        self.assertNotEqual(first.slot_id, second.slot_id)

    def test_no_free_slot(self):
        grid, slots = build_map(DEFAULT_OCCUPIED)
        self.assertIsNone(choose_slot(grid, ENTRANCE_POSE, slots, [], method="astar"))

    def test_choose_slot_hybrid_astar(self):
        grid, slots = build_map(DEFAULT_OCCUPIED)
        choice = choose_slot(grid, ENTRANCE_POSE, slots, free_ids(slots, DEFAULT_OCCUPIED))
        self.assertIsNotNone(choice)
        self.assertTrue(choice.stats["reached_goal"])
        self.assertEqual(choice.path[0], ENTRANCE_POSE)


class TestReplan(unittest.TestCase):
    def setUp(self):
        # A straight route along aisle 1 to B3's pre-park pose (19.75, 8.0, 0).
        path = [(10.0 + 0.25 * i, 8.0, 0.0) for i in range(40)]
        self.choice = SlotChoice("B3", path, {"directions": [1] * len(path)}, path[-1])
        self.start = path[0]

    def test_nothing_changed_keeps_plan(self):
        new, reason = replan(self.start, self.choice, 0, DEFAULT_OCCUPIED, method="astar")
        self.assertIs(new, self.choice)
        self.assertIsNone(reason)

    def test_slot_taken_picks_another_slot(self):
        occupied = DEFAULT_OCCUPIED | {self.choice.slot_id}
        new, reason = replan(self.start, self.choice, 0, occupied, method="astar")
        self.assertIn("occupied", reason)
        self.assertNotEqual(new.slot_id, self.choice.slot_id)
        self.assertNotIn(new.slot_id, occupied)

    def test_blocked_route_goes_round_to_same_slot(self):
        # Car on aisle 2 heading for C4; a pedestrian stands on the aisle
        # centre line in front of it, with room to pass on either side.
        grid, slots = build_map(DEFAULT_OCCUPIED)
        start = (10.0, 25.5, 0.0)
        choice = choose_slot(grid, start, slots, ["C4"], method="astar")
        ped = (17.0, 25.5, 0.6, 0.6, 0.0)
        new, reason = replan(start, choice, 0, DEFAULT_OCCUPIED, [ped], method="astar")
        self.assertIn("blocked", reason)
        self.assertEqual(new.slot_id, "C4")
        checker = CollisionChecker(build_map(DEFAULT_OCCUPIED, [ped])[0], VehicleParams())
        self.assertTrue(all(not checker.point_collides(x, y) for x, y, _ in new.path))


if __name__ == "__main__":
    unittest.main()
