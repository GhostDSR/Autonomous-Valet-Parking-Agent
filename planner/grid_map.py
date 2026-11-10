"""
Occupancy grid map in world coordinates (metres).

Part of M2 - Route Planner.

Why this file exists
--------------------
A* works on grid *cells*, but Webots (GPS, compass, the car) works in
*metres*. GridMap is the bridge between the two:

    world (x, y) in metres  <-->  grid (col, row) cell indices

Coordinate frame (matches Webots' default ENU frame, R2022a and later):
    x  -> along the ground
    y  -> along the ground, 90 deg anticlockwise from x
    z  -> up (not used: the lot is flat, so we plan in the x-y plane)
    heading (yaw) is measured anticlockwise from +x, in radians.

The map covers the rectangle [origin_x, origin_x + width] x
[origin_y, origin_y + height]. Cell (col, row) covers
    x in [origin_x + col*res, origin_x + (col+1)*res)
    y in [origin_y + row*res, origin_y + (row+1)*res)
and is stored at data[row][col]  (0 = free, 1 = occupied).
"""

import math

import numpy as np


class GridMap:
    def __init__(self, width_m, height_m, resolution=0.25, origin=(0.0, 0.0)):
        self.resolution = float(resolution)
        self.origin_x, self.origin_y = float(origin[0]), float(origin[1])
        self.cols = int(math.ceil(width_m / resolution))
        self.rows = int(math.ceil(height_m / resolution))
        self.data = np.zeros((self.rows, self.cols), dtype=np.uint8)

    # ------------------------------------------------------------------
    # Conversions between metres and cells
    # ------------------------------------------------------------------

    @property
    def width_m(self):
        return self.cols * self.resolution

    @property
    def height_m(self):
        return self.rows * self.resolution

    def world_to_grid(self, x, y):
        """Return the (col, row) of the cell that contains world point (x, y)."""
        col = int(math.floor((x - self.origin_x) / self.resolution))
        row = int(math.floor((y - self.origin_y) / self.resolution))
        return col, row

    def grid_to_world(self, col, row):
        """Return the world (x, y) of the *centre* of cell (col, row)."""
        x = self.origin_x + (col + 0.5) * self.resolution
        y = self.origin_y + (row + 0.5) * self.resolution
        return x, y

    def in_bounds(self, col, row):
        return 0 <= col < self.cols and 0 <= row < self.rows

    def is_occupied_world(self, x, y):
        """True if (x, y) is in an occupied cell. Outside the map counts as occupied."""
        col, row = self.world_to_grid(x, y)
        if not self.in_bounds(col, row):
            return True
        return bool(self.data[row, col])

    # ------------------------------------------------------------------
    # Drawing obstacles
    # ------------------------------------------------------------------

    def add_rectangle(self, cx, cy, length, width, yaw=0.0):
        """
        Mark a (possibly rotated) rectangle as occupied, e.g. a parked car.

        (cx, cy) is the centre, `length` runs along the heading `yaw`,
        `width` is perpendicular to it. A cell is marked if its centre lies
        within half a cell of the rectangle, so the obstacle is never
        under-represented by the grid.
        """
        res = self.resolution
        reach = 0.5 * math.hypot(length, width) + res
        c0, r0 = self.world_to_grid(cx - reach, cy - reach)
        c1, r1 = self.world_to_grid(cx + reach, cy + reach)
        c0, r0 = max(c0, 0), max(r0, 0)
        c1, r1 = min(c1, self.cols - 1), min(r1, self.rows - 1)
        if c0 > c1 or r0 > r1:
            return  # completely outside the map

        # World coordinates of the centres of all cells in the bounding box
        xs = self.origin_x + (np.arange(c0, c1 + 1) + 0.5) * res
        ys = self.origin_y + (np.arange(r0, r1 + 1) + 0.5) * res
        X, Y = np.meshgrid(xs, ys)

        # Rotate each cell centre into the rectangle's own frame
        dx, dy = X - cx, Y - cy
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        along = dx * cos_y + dy * sin_y
        across = -dx * sin_y + dy * cos_y

        inside = (np.abs(along) < length / 2 + res / 2) & (np.abs(across) < width / 2 + res / 2)
        self.data[r0:r1 + 1, c0:c1 + 1][inside] = 1

    def add_box(self, x_min, y_min, x_max, y_max):
        """Mark an axis-aligned box (e.g. a wall) as occupied."""
        self.add_rectangle(
            (x_min + x_max) / 2, (y_min + y_max) / 2, x_max - x_min, y_max - y_min, 0.0
        )

    # ------------------------------------------------------------------
    # Inflation
    # ------------------------------------------------------------------

    def inflate(self, radius):
        """
        Return a NEW map where every obstacle is grown by `radius` metres.

        This is the configuration-space trick: if a disc of radius r must
        not touch an obstacle, grow every obstacle by r and then treat the
        disc as a single point. A cell becomes occupied if it is within
        `radius` of an occupied cell.
        """
        out = self.copy()
        if radius <= 0:
            return out

        n = int(math.ceil(radius / self.resolution))
        occupied = self.data.astype(bool)
        grown = occupied.copy()
        for dr in range(-n, n + 1):
            for dc in range(-n, n + 1):
                if math.hypot(dr, dc) * self.resolution <= radius:
                    _shift_or(grown, occupied, dr, dc)
        out.data = grown.astype(np.uint8)
        return out

    def copy(self):
        new = GridMap.__new__(GridMap)
        new.resolution = self.resolution
        new.origin_x, new.origin_y = self.origin_x, self.origin_y
        new.cols, new.rows = self.cols, self.rows
        new.data = self.data.copy()
        return new

    def as_lists(self):
        """Plain list-of-lists copy (faster than numpy for one-cell-at-a-time lookups)."""
        return self.data.tolist()


def _shift_or(target, source, dr, dc):
    """target[r, c] |= source[r - dr, c - dc] wherever both indices are valid."""
    rows, cols = source.shape
    dst_r = slice(max(0, dr), rows + min(0, dr))
    src_r = slice(max(0, -dr), rows - max(0, dr))
    dst_c = slice(max(0, dc), cols + min(0, dc))
    src_c = slice(max(0, -dc), cols - max(0, dc))
    target[dst_r, dst_c] |= source[src_r, src_c]
