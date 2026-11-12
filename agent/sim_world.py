"""
A small offline stand-in for the Webots world, used to test the whole agent
without Webots.

It knows where the walls, parked cars and pedestrians are, and produces the
same sensor data the Webots car gives the controller:

  - a 360-degree lidar range image, in the format M1's code expects
    (samples from -FOV/2 to +FOV/2 around the car heading, inf = no return)
  - the true pose and speed of the car (GPS / compass stand-ins)

The car itself is moved by M4's kinematic bicycle model (m4_control.sim).

Only objects the lidar can see are included. The lidar sits about 0.45 m
above the ground in worlds/parking_lot.wbt, so the low island kerb is
invisible to it, but walls (1 m), parked cars (1.3 m) and pedestrians are not.
"""

import math

import numpy as np

from planner.parking_lot import (ENTRANCE_Y, LOT_HEIGHT, LOT_WIDTH, PARKED_CAR_LENGTH,
                                 PARKED_CAR_WIDTH, WALL_THICKNESS, make_slots)

LIDAR_SAMPLES = 721           # as in worlds/parking_lot.wbt
LIDAR_FOV = 2 * math.pi
LIDAR_MAX_RANGE = 40.0


def wall_boxes():
    """Outer walls as (cx, cy, length, width, yaw), matching build_parking_lot()."""
    t = WALL_THICKNESS
    boxes = []

    def box(x0, y0, x1, y1):
        boxes.append(((x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0, 0.0))

    box(0, 0, LOT_WIDTH, t)                              # bottom
    box(0, LOT_HEIGHT - t, LOT_WIDTH, LOT_HEIGHT)        # top
    box(LOT_WIDTH - t, 0, LOT_WIDTH, LOT_HEIGHT)         # right
    box(0, 0, t, ENTRANCE_Y[0])                          # left, below the entrance
    box(0, ENTRANCE_Y[1], t, LOT_HEIGHT)                 # left, above the entrance
    return boxes


def ray_box_distances(x, y, angles, boxes, max_range):
    """
    Distance along each ray from (x, y) to the nearest box (inf if none).

    Slab method: in a box's own frame the box is |u| <= a, |v| <= b. A ray
    o + t*d is inside the u-slab for t between (-a - o_u)/d_u and (a - o_u)/d_u,
    and the same for v. The ray hits the box if the two t-intervals overlap.
    All rays are handled at once with numpy.
    """
    dx, dy = np.cos(angles), np.sin(angles)
    best = np.full(len(angles), np.inf)
    for cx, cy, length, width, yaw in boxes:
        c, s = math.cos(yaw), math.sin(yaw)
        ou = c * (x - cx) + s * (y - cy)          # ray origin in the box frame
        ov = -s * (x - cx) + c * (y - cy)
        du = c * dx + s * dy                      # ray directions in the box frame
        dv = -s * dx + c * dy
        with np.errstate(divide="ignore", invalid="ignore"):
            t1u, t2u = (-length / 2 - ou) / du, (length / 2 - ou) / du
            t1v, t2v = (-width / 2 - ov) / dv, (width / 2 - ov) / dv
        inside_u = abs(ou) <= length / 2
        inside_v = abs(ov) <= width / 2
        enter_u, exit_u = np.minimum(t1u, t2u), np.maximum(t1u, t2u)
        enter_v, exit_v = np.minimum(t1v, t2v), np.maximum(t1v, t2v)
        # A ray parallel to a slab is inside it for every t, or for none.
        enter_u = np.where(du == 0, -np.inf if inside_u else np.inf, enter_u)
        exit_u = np.where(du == 0, np.inf if inside_u else -np.inf, exit_u)
        enter_v = np.where(dv == 0, -np.inf if inside_v else np.inf, enter_v)
        exit_v = np.where(dv == 0, np.inf if inside_v else -np.inf, exit_v)
        t_enter = np.maximum(enter_u, enter_v)
        t_exit = np.minimum(exit_u, exit_v)
        hit = (t_exit >= np.maximum(t_enter, 0.0))
        dist = np.where(hit, np.maximum(t_enter, 0.0), np.inf)
        best = np.minimum(best, dist)
    best[best >= max_range] = np.inf
    return best


class SimWorld:
    """The 'real' lot: what is actually there, whatever the agent believes."""

    def __init__(self, occupied, pedestrians=None):
        self.slots = make_slots()
        self.occupied = set(occupied)
        # pedestrian name -> (x, y); about 0.5 x 0.5 m seen from above
        self.pedestrians = dict(pedestrians or {})

    def park_car(self, slot_id):
        """Another car takes this slot (used to test replanning)."""
        self.occupied.add(slot_id)

    def boxes(self):
        boxes = wall_boxes()
        for sid in self.occupied:
            s = self.slots[sid]
            boxes.append((s.x, s.y, PARKED_CAR_LENGTH, PARKED_CAR_WIDTH, s.yaw))
        for px, py in self.pedestrians.values():
            boxes.append((px, py, 0.5, 0.5, 0.0))
        return boxes

    def lidar_scan(self, pose):
        """Range image for a lidar at the rear axle, same layout as M1 expects."""
        x, y, yaw = pose
        step = LIDAR_FOV / (LIDAR_SAMPLES - 1)
        angles = yaw - LIDAR_FOV / 2 + step * np.arange(LIDAR_SAMPLES)
        return ray_box_distances(x, y, angles, self.boxes(), LIDAR_MAX_RANGE).tolist()

    def car_hits_obstacle(self, pose, length, width, rear_overhang):
        """True if the car's rectangle overlaps any obstacle (sampled outline check)."""
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        pts = []
        for along in np.linspace(-rear_overhang, length - rear_overhang, 24):
            for across in (-width / 2, width / 2):
                pts.append((x + along * c - across * s, y + along * s + across * c))
        for across in np.linspace(-width / 2, width / 2, 10):
            for along in (-rear_overhang, length - rear_overhang):
                pts.append((x + along * c - across * s, y + along * s + across * c))
        for cx, cy, bl, bw, byaw in self.boxes():
            bc, bs = math.cos(byaw), math.sin(byaw)
            for px, py in pts:
                u = bc * (px - cx) + bs * (py - cy)
                v = -bs * (px - cx) + bc * (py - cy)
                if abs(u) < bl / 2 and abs(v) < bw / 2:
                    return True
        return False
