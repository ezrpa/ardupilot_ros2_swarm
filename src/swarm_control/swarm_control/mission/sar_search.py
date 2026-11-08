#!/usr/bin/env python3
"""Coordinated grid search for SAR: BFS / DFS over a shared cell grid.

The search area is a grid of cells (one camera frame each). Drones start from
separate launch points and each runs its own graph search over the grid's
4-connected cell graph; they coordinate only through CLAIMS on the shared
call channel (logs/sar/claims/, one O_EXCL file per cell), so no cell is ever
searched twice and no central planner is needed.

  bfs  "nearest unclaimed cell": from the drone's CURRENT cell, breadth-first
       search the grid graph and take the closest cell nobody has claimed.
       Drones sweep outward from their launch corners like expanding rings
       and naturally split the area where their fronts meet. Ties go to the
       cell continuing the current heading (fewer turns, straighter legs).
  dfs  depth-first from the launch cell: keep going to an unvisited
       neighbour (same direction preferred, so the walk lays down long
       boustrophedon-like runs), backtracking along the drone's own stack
       when boxed in by searched / claimed cells.

Both stop when no unclaimed cell remains reachable. ROS-free; unit-tested in
tests/test_sar_search.py.
"""
import json
import math
import os
import time
from collections import deque

from swarm_control.core import config, geo

NEIGHBOURS = ((1, 0), (0, 1), (-1, 0), (0, -1))   # N, E, S, W in (row, col)
ALGORITHMS = ("bfs", "dfs")


class Grid:
    """size_m square centred on (origin_lat, origin_lon), n x n cells.
    Row 0 is the SOUTH edge, col 0 the WEST edge."""

    def __init__(self, origin_lat, origin_lon, size_m=120.0, n=7):
        self.lat0, self.lon0 = origin_lat, origin_lon
        self.size, self.n = float(size_m), int(n)
        self.cell_m = self.size / self.n
        self.frame = geo.LocalFrame(origin_lat, origin_lon)

    def cells(self):
        return [(r, c) for r in range(self.n) for c in range(self.n)]

    def contains(self, cell):
        return 0 <= cell[0] < self.n and 0 <= cell[1] < self.n

    def center_ne(self, cell):
        r, c = cell
        half = self.size / 2.0
        return (-half + (r + 0.5) * self.cell_m,
                -half + (c + 0.5) * self.cell_m)

    def center_latlon(self, cell):
        return self.frame.to_latlon(*self.center_ne(cell))

    def corners_latlon(self, cell):
        n, e = self.center_ne(cell)
        h = self.cell_m / 2.0
        return [self.frame.to_latlon(n + dn, e + de)
                for dn, de in ((h, -h), (h, h), (-h, h), (-h, -h))]

    def cell_at(self, lat, lon):
        """Nearest cell to a position (clamped into the grid)."""
        n, e = self.frame.to_ne(lat, lon)
        half = self.size / 2.0
        r = int(math.floor((n + half) / self.cell_m))
        c = int(math.floor((e + half) / self.cell_m))
        return (min(max(r, 0), self.n - 1), min(max(c, 0), self.n - 1))

    def neighbours(self, cell, first_dir=None):
        dirs = list(NEIGHBOURS)
        if first_dir in dirs:          # prefer continuing straight
            dirs.remove(first_dir)
            dirs.insert(0, first_dir)
        for dr, dc in dirs:
            nxt = (cell[0] + dr, cell[1] + dc)
            if self.contains(nxt):
                yield nxt, (dr, dc)


def bfs_nearest(grid, start, is_free, heading=None):
    """Breadth-first search from `start` over the cell graph; returns the
    closest cell (fewest grid steps) for which is_free(cell) is True, or None.
    `heading` (dr, dc) orders neighbours so equal-distance ties continue
    straight. `start` itself counts (distance 0)."""
    if is_free(start):
        return start
    seen = {start}
    q = deque([(start, heading)])
    while q:
        cell, d = q.popleft()
        for nxt, step in grid.neighbours(cell, d):
            if nxt in seen:
                continue
            if is_free(nxt):
                return nxt
            seen.add(nxt)
            q.append((nxt, step))
    return None


class SearchPlanner:
    """Per-drone planner. next_cell() returns the next cell this drone has
    CLAIMED (so it is exclusively its to search), or None when done."""

    def __init__(self, grid, start_cell, claim, is_claimed, algorithm="bfs"):
        if algorithm not in ALGORITHMS:
            raise ValueError(f"unknown search algorithm {algorithm!r}")
        self.grid, self.algo = grid, algorithm
        self.claim, self.is_claimed = claim, is_claimed
        self.current = start_cell
        self.heading = None
        self.visited = []                 # cells this drone searched
        self._stack = [(start_cell, None)]  # dfs: (cell, arrival direction)

    def _step_dir(self, a, b):
        dr, dc = b[0] - a[0], b[1] - a[1]
        if abs(dr) + abs(dc) == 1:
            return (dr, dc)
        return None

    def next_cell(self):
        while True:
            if self.algo == "bfs":
                cell = bfs_nearest(self.grid, self.current,
                                   lambda c: not self.is_claimed(c),
                                   self.heading)
                if cell is None:
                    return None
            else:
                cell = self._dfs_pop()
                if cell is None:
                    return None
            if self.claim(cell):          # lost a race: pick again
                self.heading = self._step_dir(self.current, cell) \
                    or self.heading
                self.current = cell
                self.visited.append(cell)
                return cell

    def _dfs_pop(self):
        while self._stack:
            cell, arrived = self._stack[-1]
            if not self.is_claimed(cell):
                return cell
            # cell done (by us or someone else): descend to a free neighbour,
            # preferring to keep the direction we arrived in
            for nxt, step in self.grid.neighbours(cell, arrived):
                if not self.is_claimed(nxt):
                    self._stack.append((nxt, step))
                    break
            else:
                self._stack.pop()         # dead end: backtrack
        # own subtree exhausted; jump to the nearest free cell anywhere
        jump = bfs_nearest(self.grid, self.current,
                           lambda c: not self.is_claimed(c), self.heading)
        if jump is not None:
            self._stack.append((jump, None))
        return jump


class ClaimBoard:
    """Cell claims on the shared call channel: <logs>/sar/claims/r<R>c<C>.json,
    created with O_EXCL so exactly one drone owns each cell."""

    def __init__(self, logs_dir, grid):
        self.dir = os.path.join(logs_dir, "sar", "claims")
        os.makedirs(self.dir, exist_ok=True)
        self.grid = grid

    def _path(self, cell):
        return os.path.join(self.dir, f"r{cell[0]}c{cell[1]}.json")

    def is_claimed(self, cell):
        return os.path.exists(self._path(cell))

    def claim(self, cell, instance, **extra):
        try:
            fd = os.open(self._path(cell),
                         os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o644)
        except FileExistsError:
            return False
        rec = {"row": cell[0], "col": cell[1], "instance": instance,
               "unix_time": time.time(),
               "corners": self.grid.corners_latlon(cell),
               "center": self.grid.center_latlon(cell), **extra}
        with os.fdopen(fd, "w") as f:
            json.dump(rec, f)
        return True

    def all_claims(self):
        out = []
        for name in os.listdir(self.dir):
            if name.endswith(".json"):
                try:
                    with open(os.path.join(self.dir, name)) as f:
                        out.append(json.load(f))
                except (OSError, json.JSONDecodeError):
                    continue  # mid-write
        return out


# Separate SAR launch points are part of the deployment contract (the drone
# container spawns SITL there): owned by core/config.py.
launch_points = config.launch_points
