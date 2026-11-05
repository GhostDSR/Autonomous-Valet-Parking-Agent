"""
A* path planner on a 2D occupancy grid.

Part of M2 - Route Planner (Autonomous Valet Parking Agent, AISR P13).

What A* does
------------
A* searches a graph for the cheapest path from a start node to a goal node.
For every node n it keeps:

    g(n) = the true cost of the best path found so far from start to n
    h(n) = a heuristic: an estimate of the remaining cost from n to the goal
    f(n) = g(n) + h(n)   (estimated total cost of a path through n)

It always expands the node with the smallest f(n) next (using a priority
queue). If h(n) never overestimates the real remaining cost ("admissible"),
the first time A* pops the goal, the path it found is the shortest one.

Grid conventions used here
--------------------------
- The grid is a list of rows (or a 2D numpy array): grid[y][x].
  0 = free cell, 1 = obstacle (a parked car, a wall, a pillar ...).
- A cell is written as (x, y): x = column index, y = row index.
- Moves are 8-connected by default (up/down/left/right + diagonals).
  Straight moves cost 1, diagonal moves cost sqrt(2).
- Diagonal moves are not allowed to "cut a corner" between two obstacles,
  because a real car could not squeeze through that gap.

NOTE: cell (x, y) is a grid index, not a Webots position in metres.
Converting between the two (resolution, origin, axis directions) still has
to be agreed with M1 (World and Sensors).
"""

import heapq
import math
import time

# Neighbour offsets (dx, dy) and their move costs.
STRAIGHT_MOVES = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0)]
DIAGONAL_MOVES = [
    (1, 1, math.sqrt(2)),
    (1, -1, math.sqrt(2)),
    (-1, 1, math.sqrt(2)),
    (-1, -1, math.sqrt(2)),
]


# ---------------------------------------------------------------------------
# Heuristics
# ---------------------------------------------------------------------------

def manhattan(a, b):
    """Exact shortest distance on a 4-connected grid with no obstacles."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def octile(a, b):
    """
    Exact shortest distance on an 8-connected grid with no obstacles.

    Move diagonally until you line up with the goal on one axis, then go
    straight: cost = sqrt(2) * min(dx, dy) + (max(dx, dy) - min(dx, dy)).
    """
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    return (math.sqrt(2) - 1) * min(dx, dy) + max(dx, dy)


# ---------------------------------------------------------------------------
# Grid helpers
# ---------------------------------------------------------------------------

def in_bounds(grid, cell):
    x, y = cell
    return 0 <= y < len(grid) and 0 <= x < len(grid[0])


def is_free(grid, cell):
    x, y = cell
    return grid[y][x] == 0


def get_neighbours(grid, cell, allow_diagonal=True):
    """Yield (neighbour_cell, move_cost) for every legal move from `cell`."""
    x, y = cell
    moves = STRAIGHT_MOVES + (DIAGONAL_MOVES if allow_diagonal else [])

    for dx, dy, cost in moves:
        nxt = (x + dx, y + dy)
        if not in_bounds(grid, nxt) or not is_free(grid, nxt):
            continue

        # No corner cutting: for a diagonal move, both side cells must be free.
        if dx != 0 and dy != 0:
            if not is_free(grid, (x + dx, y)) or not is_free(grid, (x, y + dy)):
                continue

        yield nxt, cost


def reconstruct_path(came_from, goal):
    """Walk back from the goal through the parent links to rebuild the path."""
    path = [goal]
    while path[-1] in came_from:
        path.append(came_from[path[-1]])
    path.reverse()
    return path


# ---------------------------------------------------------------------------
# A* search
# ---------------------------------------------------------------------------

def astar(grid, start, goal, allow_diagonal=True):
    """
    Run A* and return (path, stats).

    path  : list of (x, y) cells from start to goal, or [] if no path exists.
    stats : dict with 'cost', 'nodes_expanded' and 'time_s'
            (useful later for the A* vs Hybrid A* comparison).
    """
    t0 = time.perf_counter()
    stats = {"cost": math.inf, "nodes_expanded": 0, "time_s": 0.0}

    for name, cell in (("start", start), ("goal", goal)):
        if not in_bounds(grid, cell):
            raise ValueError(f"{name} {cell} is outside the grid")
        if not is_free(grid, cell):
            raise ValueError(f"{name} {cell} is inside an obstacle")

    heuristic = octile if allow_diagonal else manhattan

    # Priority queue of (f, g, cell). heapq always pops the smallest f first;
    # on a tie in f, the smaller g comes first.
    open_heap = [(heuristic(start, goal), 0.0, start)]
    g_cost = {start: 0.0}   # best known cost from start to each cell
    came_from = {}          # parent of each cell on its best known path
    closed = set()          # cells already expanded (their g is final)

    while open_heap:
        _, g, cell = heapq.heappop(open_heap)

        # The same cell can be in the heap several times with different
        # costs; skip the stale copies once it has been expanded.
        if cell in closed:
            continue
        closed.add(cell)
        stats["nodes_expanded"] += 1

        if cell == goal:
            stats["cost"] = g
            stats["time_s"] = time.perf_counter() - t0
            return reconstruct_path(came_from, goal), stats

        for nxt, move_cost in get_neighbours(grid, cell, allow_diagonal):
            new_g = g + move_cost
            # Only keep this route to `nxt` if it is cheaper than any found so far.
            if new_g < g_cost.get(nxt, math.inf):
                g_cost[nxt] = new_g
                came_from[nxt] = cell
                heapq.heappush(open_heap, (new_g + heuristic(nxt, goal), new_g, nxt))

    # Heap is empty and the goal was never reached: no path exists.
    stats["time_s"] = time.perf_counter() - t0
    return [], stats


def plan_path(grid, start, goal, allow_diagonal=True):
    """
    Public interface for the rest of the team (e.g. the Webots controller).

    plan_path(grid, (x, y), (x, y)) -> list of (x, y) cells, [] if unreachable.
    """
    path, _ = astar(grid, start, goal, allow_diagonal)
    return path


# ---------------------------------------------------------------------------
# Demo: a small test parking lot + matplotlib plot
# ---------------------------------------------------------------------------

def make_test_lot():
    """
    Build a small toy parking lot (30 x 20 cells). NOT the real Webots lot.

    Layout: outer walls, an entrance gap on the left wall, two rows of
    parking bays (top and bottom) separated by a central driving aisle,
    and some bays occupied by parked cars.
    """
    width, height = 30, 20
    grid = [[0] * width for _ in range(height)]

    # Outer walls
    for x in range(width):
        grid[0][x] = 1
        grid[height - 1][x] = 1
    for y in range(height):
        grid[y][0] = 1
        grid[y][width - 1] = 1

    # Entrance: a gap in the left wall, in the middle of the aisle
    for y in (9, 10):
        grid[y][0] = 0

    # Bay dividers: thin vertical lines every 3 cells, along top and bottom
    for x in range(4, 27, 3):
        for y in range(1, 6):          # bottom row of bays
            grid[y][x] = 1
        for y in range(14, 19):        # top row of bays
            grid[y][x] = 1

    # Parked cars: fill some bays (each bay is 2 cells wide between dividers)
    occupied_bottom = [5, 8, 14, 20]   # x of the left cell of each bay
    occupied_top = [5, 11, 17, 20]
    for bx in occupied_bottom:
        for y in range(1, 5):
            grid[y][bx] = grid[y][bx + 1] = 1
    for bx in occupied_top:
        for y in range(15, 19):
            grid[y][bx] = grid[y][bx + 1] = 1

    # A pillar in the middle of the aisle, so the route has to go round it
    for x in range(13, 16):
        for y in range(9, 11):
            grid[y][x] = 1

    start = (0, 9)    # entrance
    goal = (24, 16)   # a free bay at the top right
    return grid, start, goal


def plot_path(grid, path, start, goal, title="A* path", save_path=None, show=True):
    """Draw the grid (obstacles in dark grey), the path, the start and the goal."""
    import matplotlib.pyplot as plt  # imported here so the planner itself needs no matplotlib

    fig, ax = plt.subplots(figsize=(10, 7))
    # origin="lower" puts y = 0 at the bottom, like a normal x-y plot.
    ax.imshow(grid, cmap="Greys", origin="lower", vmin=0, vmax=1.4)

    if path:
        xs = [c[0] for c in path]
        ys = [c[1] for c in path]
        ax.plot(xs, ys, "-o", color="tab:blue", markersize=3, linewidth=2, label="A* path")

    ax.plot(*start, "s", color="tab:green", markersize=12, label="start (entrance)")
    ax.plot(*goal, "*", color="tab:red", markersize=16, label="goal (free slot)")

    ax.set_title(title)
    ax.set_xlabel("x (cell)")
    ax.set_ylabel("y (cell)")
    ax.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0))
    fig.tight_layout()

    if save_path:
        fig.savefig(save_path, dpi=150)
        print(f"Saved plot to {save_path}")
    if show:
        plt.show()
    plt.close(fig)


def main():
    import argparse
    import os

    parser = argparse.ArgumentParser(description="Run A* on a small test parking lot.")
    parser.add_argument("--no-show", action="store_true", help="save the plot without opening a window")
    parser.add_argument("--four-connected", action="store_true", help="disallow diagonal moves")
    args = parser.parse_args()

    grid, start, goal = make_test_lot()
    allow_diagonal = not args.four_connected
    path, stats = astar(grid, start, goal, allow_diagonal)

    if path:
        print(f"Path found: {len(path)} cells, cost = {stats['cost']:.2f}")
    else:
        print("No path found.")
    print(f"Nodes expanded: {stats['nodes_expanded']}, time: {stats['time_s'] * 1000:.2f} ms")

    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    out = os.path.join(repo_root, "results", "astar_test_lot.png")
    mode = "8-connected" if allow_diagonal else "4-connected"
    plot_path(grid, path, start, goal, title=f"A* on test parking lot ({mode})",
              save_path=out, show=not args.no_show)


if __name__ == "__main__":
    main()
