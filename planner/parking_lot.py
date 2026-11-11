"""
Parking-lot layout used by the planner, in Webots world coordinates (metres).

Part of M2 - Route Planner.

!!! Every number in this file is a PLACEHOLDER until it is agreed with M1 !!!
The Webots world (worlds/parking_lot.wbt) and this file must describe the
same lot: same origin, same axis directions, same bay positions and ids.

Layout (top view, x to the right, y up, origin at the bottom-left corner):

    y=34 +--------------------------------------------------+
 y=33.5  |        +-----------------------------+           |
         |        | D0 D1 D2 ... D9 (nose -y)   |           |   row D
 y=28.5  |        +-----------------------------+           |
         |  left  |          aisle 2            |   right   |
    y=22 |  lane  +-----------------------------+   lane    |
         |        | C0 C1 C2 ... C9 (nose +y)   |           |   row C  } island
    y=17 |        | B0 B1 B2 ... B9 (nose -y)   |           |   row B  }
    y=12 |        +-----------------------------+           |
    y=11 =  entrance       aisle 1                          |
    y=6  =  (gap in wall)                                   |
  y=5.5  |        +-----------------------------+           |
         |        | A0 A1 A2 ... A9 (nose +y)   |           |   row A
  y=0.5  |        +-----------------------------+           |
    y=0  +--------------------------------------------------+
        x=0      x=8                          x=33         x=40

Bays are 2.5 m wide and 5 m deep. Bay lines are only paint, so they are
not obstacles; parked cars and walls are. Rows B and C sit back to back
on an island with a kerb along y = 17 between them, so a car cannot cut
across the island through two empty bays.

"Nose +y" means a car parked in that bay faces +y, i.e. it reversed in and
its front points at the aisle (the usual way to valet-park).
"""

import math
from dataclasses import dataclass

from planner.grid_map import GridMap

# ---- Lot geometry (metres) - PLACEHOLDERS, agree with M1 ----
LOT_WIDTH = 40.0
LOT_HEIGHT = 34.0
WALL_THICKNESS = 0.25
ENTRANCE_Y = (6.0, 11.0)            # gap in the left wall (x = 0)

BAY_WIDTH = 2.5
BAY_DEPTH = 5.0
BAYS_PER_ROW = 10
BAYS_X_START = 8.0                  # bays run from x = 8 to x = 33

ISLAND_KERB_Y = 17.0                # kerb between rows B and C
ISLAND_KERB_THICKNESS = 0.2

PARKED_CAR_LENGTH = 4.5
PARKED_CAR_WIDTH = 1.9

# (row name, y of the row's bottom edge, heading of a parked car, y of the aisle centre)
# Rows A and D stand 0.5 m off the outer wall (0.25 m from its inner face):
# a 4.6 m car centred in a 5 m bay ends 0.2 m from the bay's back line, and
# with the bay starting at the wall itself its rear would be inside the wall.
ROWS = [
    ("A", 0.5, math.pi / 2, 8.5),
    ("B", 12.0, -math.pi / 2, 8.5),
    ("C", 17.0, math.pi / 2, 25.5),
    ("D", 28.5, -math.pi / 2, 25.5),
]

# Where the car starts: just inside the entrance, facing into the lot.
# (x, y, heading) of the rear-axle centre.
ENTRANCE_POSE = (2.0, 8.5, 0.0)

# A fixed example of which bays are taken. In the real system the free-slot
# detector (M1) decides this.
DEFAULT_OCCUPIED = {
    "A0", "A1", "A3", "A4", "A6", "A8",
    "B1", "B2", "B5", "B7", "B9",
    "C0", "C2", "C3", "C6", "C8",
    "D1", "D2", "D4", "D5", "D7", "D9",
}


@dataclass(frozen=True)
class Slot:
    slot_id: str
    x: float          # centre of the bay
    y: float
    yaw: float        # heading of a car parked in this bay
    aisle_y: float    # centre line of the aisle the bay opens onto


def make_slots():
    """Return {slot_id: Slot} for every bay in the lot."""
    slots = {}
    for name, y_bottom, yaw, aisle_y in ROWS:
        for i in range(BAYS_PER_ROW):
            sid = f"{name}{i}"
            x = BAYS_X_START + (i + 0.5) * BAY_WIDTH
            y = y_bottom + BAY_DEPTH / 2
            slots[sid] = Slot(sid, x, y, yaw, aisle_y)
    return slots


def build_parking_lot(occupied=None, resolution=0.25):
    """
    Build the occupancy grid of the lot.

    occupied   : set of slot ids that contain a parked car
                 (defaults to DEFAULT_OCCUPIED).
    resolution : grid cell size in metres.

    Returns (grid_map, slots).
    """
    if occupied is None:
        occupied = DEFAULT_OCCUPIED
    slots = make_slots()
    unknown = set(occupied) - set(slots)
    if unknown:
        raise ValueError(f"unknown slot ids: {sorted(unknown)}")

    grid = GridMap(LOT_WIDTH, LOT_HEIGHT, resolution)
    t = WALL_THICKNESS

    # Outer walls, with a gap in the left wall for the entrance
    grid.add_box(0, 0, LOT_WIDTH, t)                              # bottom
    grid.add_box(0, LOT_HEIGHT - t, LOT_WIDTH, LOT_HEIGHT)        # top
    grid.add_box(LOT_WIDTH - t, 0, LOT_WIDTH, LOT_HEIGHT)         # right
    grid.add_box(0, 0, t, ENTRANCE_Y[0])                          # left, below entrance
    grid.add_box(0, ENTRANCE_Y[1], t, LOT_HEIGHT)                 # left, above entrance

    # Kerb down the middle of the island
    k = ISLAND_KERB_THICKNESS / 2
    grid.add_box(BAYS_X_START, ISLAND_KERB_Y - k,
                 BAYS_X_START + BAYS_PER_ROW * BAY_WIDTH, ISLAND_KERB_Y + k)

    # Parked cars, centred in their bays and lined up with the bay
    for sid in occupied:
        s = slots[sid]
        grid.add_rectangle(s.x, s.y, PARKED_CAR_LENGTH, PARKED_CAR_WIDTH, s.yaw)

    return grid, slots


def free_slots(slots, occupied=None):
    """Ids of all free bays, sorted."""
    if occupied is None:
        occupied = DEFAULT_OCCUPIED
    return sorted(sid for sid in slots if sid not in occupied)


def pre_park_poses(slot, offset=3.0, standoff=4.0):
    """
    Candidate goal poses for the route planner.

    The planner does NOT drive into the bay itself. It brings the car to a
    "pre-park" pose on the aisle, just past the bay, facing along the aisle.
    From there the reactive parking manoeuvre (M3) reverses the car into the
    bay. This is the deliberative / reactive split from the problem statement.

    Two candidates are returned, one for each direction of travel along the
    aisle (the cheaper one is chosen by the route planner):
      - driving in +x : stop `offset` m past the bay, heading 0
      - driving in -x : stop `offset` m before the bay, heading pi

    Sideways, the car stops `standoff` m out from the bay's entrance line.
    Why 4 m: M3 reverses in on a quarter circle of radius
    R = L / tan(0.85 * max_steer) = 6.19 m that starts where the car stands.
    The parked car in the next bay is on the inside of that turn, so all of
    it must stay within the innermost circle the body sweeps, radius
    R - width/2 = 5.24 m. Stopping 3.5 m out (the aisle centre line) leaves
    a gap of zero to that car's far corner; 4 m out leaves about 0.2 m, with
    the outer side of the swing still clear of the row across the aisle.
    Worked out for the placeholder car; recheck if the car changes.
    """
    towards_aisle = 1.0 if slot.aisle_y > slot.y else -1.0
    y = slot.y + towards_aisle * (BAY_DEPTH / 2 + standoff)
    return [
        (slot.x + offset, y, 0.0),
        (slot.x - offset, y, math.pi),
    ]
