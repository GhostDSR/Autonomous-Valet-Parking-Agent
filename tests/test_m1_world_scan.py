"""Slot detection against the geometry of worlds/parking_lot.wbt.

CI cannot run Webots, so this ray-casts a 721-sample, 360-degree lidar scan
against the walls and parked cars read straight from the world file (the
things the lidar plane at z = 0.58 m can hit; the 0.24 m island kerb and the
paint lines are below it) and checks what classify_slots_from_scan makes of it.
"""
import math
import os
import re
import unittest

from controllers.valet_controller.m1_sensors import classify_slots_from_scan
from planner.parking_lot import make_slots

WORLD = os.path.join(os.path.dirname(__file__), "..", "worlds", "parking_lot.wbt")
SAMPLES = 721
MAX_RANGE = 40.0


def load_obstacles():
    """[(name, cx, cy, yaw, half_x, half_y)] for walls and parked cars."""
    with open(WORLD) as handle:
        text = handle.read()
    pattern = re.compile(
        r'Solid \{\n  translation (\S+) (\S+) \S+\n(?:  rotation 0 0 1 (\S+)\n)?'
        r'  name "([^"]+)".*?boundingObject Box \{ size (\S+) (\S+) \S+ \}', re.S)
    out = []
    for x, y, rot, name, sx, sy in pattern.findall(text):
        if name.endswith("_wall") or name.startswith("left_wall") or name.startswith("parked_car_"):
            out.append((name, float(x), float(y), float(rot or 0.0), float(sx) / 2, float(sy) / 2))
    return out


def cast(obstacles, origin, angle):
    """Distance to the nearest obstacle along a ray (slab test per oriented box)."""
    ox, oy = origin
    dx, dy = math.cos(angle), math.sin(angle)
    best = math.inf
    for _name, cx, cy, yaw, hx, hy in obstacles:
        c, s = math.cos(yaw), math.sin(yaw)
        rx, ry = ox - cx, oy - cy
        local_o = (c * rx + s * ry, -s * rx + c * ry)
        local_d = (c * dx + s * dy, -s * dx + c * dy)
        t0, t1, hit = 0.0, math.inf, True
        for o, d, h in ((local_o[0], local_d[0], hx), (local_o[1], local_d[1], hy)):
            if abs(d) < 1e-12:
                if abs(o) > h:
                    hit = False
                    break
                continue
            a, b = (-h - o) / d, (h - o) / d
            if a > b:
                a, b = b, a
            t0, t1 = max(t0, a), min(t1, b)
            if t0 > t1:
                hit = False
                break
        if hit and t1 >= 0 and t0 > 0.05:
            best = min(best, t0)
    return best


def scan(obstacles, pose):
    x, y, yaw = pose
    ranges = []
    for i in range(SAMPLES):
        angle = -math.pi + i * 2 * math.pi / (SAMPLES - 1) + yaw
        d = cast(obstacles, (x, y), angle)
        ranges.append(d if d < MAX_RANGE else float("inf"))
    return ranges


class WorldScanTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.obstacles = load_obstacles()
        cls.slots = make_slots()
        cls.occupied = {n[len("parked_car_"):] for n, *_ in cls.obstacles if n.startswith("parked_car_")}
        cls.cache = {}

    def classify(self, pose):
        if pose not in self.cache:
            self.cache[pose] = classify_slots_from_scan(scan(self.obstacles, pose), pose, self.slots)
        return self.cache[pose]

    def test_no_free_bay_is_ever_reported_occupied(self):
        # From the middle of each aisle at several x positions, facing along the aisle.
        for y in (8.5, 25.5):
            for x in (3.0, 8.0, 14.0, 20.0, 26.0, 32.0, 37.0):
                result = self.classify((x, y, 0.0))
                for sid, state in result.items():
                    if state == "occupied":
                        self.assertIn(sid, self.occupied, "free bay %s read occupied from (%g, %g)" % (sid, x, y))

    def test_wall_side_bays_read_free_from_the_aisle_in_front_of_them(self):
        # Rows A (wall at y = 0) and D (wall at y = 34): every free bay must be
        # seen as free, not occupied and not unknown, from directly in front.
        for sid, slot in self.slots.items():
            if sid[0] not in "AD" or sid in self.occupied:
                continue
            state = self.classify((slot.x, slot.aisle_y, 0.0))[sid]
            self.assertEqual(state, "free", sid)

    def test_parked_cars_read_occupied_from_the_aisle_in_front_of_them(self):
        for sid in self.occupied:
            slot = self.slots[sid]
            state = self.classify((slot.x, slot.aisle_y, 0.0))[sid]
            self.assertEqual(state, "occupied", sid)

    def test_free_bays_between_two_parked_cars_are_not_covered_by_them(self):
        # A2 sits between A1 and A3; with unrotated 4.5 m-wide car boxes both
        # neighbours used to overlap it.
        for sid in ("A2", "A5", "A7", "A9", "D0", "D3", "D6", "D8"):
            slot = self.slots[sid]
            self.assertEqual(self.classify((slot.x, slot.aisle_y, 0.0))[sid], "free", sid)


if __name__ == "__main__":
    unittest.main()
