"""Scoring frontiers by what a visit would reveal, and tracking frontiers
that a visit did not shrink.

Works on the raw OccupancyGrid data (no ROS), so it can be tested with a
hand-made grid, like open_space.py and obstacle_sensing.py.
"""

import math
from collections import deque

import numpy as np

# OccupancyGrid values: 0 free, 100 occupied, -1 unknown. At or above this
# counts as a wall (map_server's default occupied threshold of 0.65).
OCCUPIED_THRESHOLD = 65
UNKNOWN = -1


def _add_visible_unknown(data, width, height, gx, gy, range_cells, num_rays,
                         seen):
    """Add to `seen` the unknown cells the rays from cell (gx, gy) reach."""
    for i in range(num_rays):
        angle = 2.0 * math.pi * i / num_rays
        dx, dy = math.cos(angle), math.sin(angle)
        for step in range(1, range_cells + 1):
            x = int(math.floor(gx + 0.5 + dx * step))
            y = int(math.floor(gy + 0.5 + dy * step))
            if not (0 <= x < width and 0 <= y < height):
                break
            value = data[y * width + x]
            if value >= OCCUPIED_THRESHOLD:
                break
            if value == UNKNOWN:
                seen.add((x, y))


def count_revealed_unknown(data, width, height, gx, gy, range_cells,
                           num_rays):
    """How many unknown cells the lidar would see from cell (gx, gy).

    Casts `num_rays` evenly spaced rays out to `range_cells`. A ray stops at
    a wall or the grid edge, and passes through free and unknown cells (the
    lidar can't know what unknown space holds until it looks). Each unknown
    cell counts once, however many rays reach it.

    This is an estimate of a visit's value: it ignores what the robot sees
    on the way there, and rays this sparse miss cells far out. Both affect
    every candidate alike, which is what ranking needs.
    """
    seen = set()
    _add_visible_unknown(data, width, height, gx, gy, range_cells, num_rays,
                         seen)
    return len(seen)


def count_unknown_behind(data, width, height, cells, range_cells, num_rays):
    """How many unknown cells the lidar could see from any of `cells`.

    The same rays as count_revealed_unknown, cast from every cell of a
    frontier group; each unknown cell counts once. A few gaps behind
    furniture see little; an opening into an unseen room sees a lot, even
    when sparse lidar broke its edge into crumbs (#137).
    """
    seen = set()
    for gx, gy in cells:
        _add_visible_unknown(data, width, height, gx, gy, range_cells,
                             num_rays, seen)
    return len(seen)


def group_frontier_pieces(pieces, link_cells):
    """Join frontier pieces whose cells lie within `link_cells` of each other
    (straight-line distance, in cells), chaining through pieces in between.

    Sparse lidar leaves wall specks along an edge it barely saw, so one
    opening can arrive as many tiny pieces (#137). Returns the groups, each
    a list of cells.
    """
    owner = {}
    for index, piece in enumerate(pieces):
        for cell in piece:
            owner[cell] = index
    parent = list(range(len(pieces)))

    def root(index):
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return index

    reach = [
        (dx, dy)
        for dx in range(-link_cells, link_cells + 1)
        for dy in range(-link_cells, link_cells + 1)
        if dx * dx + dy * dy <= link_cells * link_cells
    ]
    for (x, y), index in owner.items():
        for dx, dy in reach:
            other = owner.get((x + dx, y + dy))
            if other is not None and other != index:
                parent[root(other)] = root(index)

    groups = {}
    for index, piece in enumerate(pieces):
        groups.setdefault(root(index), []).extend(piece)
    return list(groups.values())


def visit_utility(revealed_cells, travel_m, cells_per_meter):
    """Value of a candidate: unknown cells it would reveal, minus a cost of
    `cells_per_meter` for every meter of travel, so a slightly smaller gain
    nearby beats a slightly larger one across the map."""
    return revealed_cells - cells_per_meter * travel_m


def world_key(x, y, resolution):
    """World-cell index of the cell whose floor corner is (x, y).

    Pass that corner (map origin + cell index * resolution), not the cell
    center. A center sits at k + 0.5 cells, and round() sends halves to the
    nearest even number, so neighboring cells share a key (296 frontier
    cells became 247 keys).

    The corner is a whole number of cells. round() snaps it back onto that
    integer when float error nudges it; floor() would drop a ratio that
    lands just under the integer into the previous cell, and those two
    cells would share a key again.
    """
    return (int(round(x / resolution)), int(round(y / resolution)))


def remaining_size(target_keys, clusters):
    """Cells left of a target frontier after the map changed.

    `target_keys` are the world keys of the frontier as it was; `clusters`
    is the current frontiers, each a set of world keys. A frontier can split
    or shift a little, so every current cluster that shares any cell with
    the target counts toward it. 0 means the frontier is gone (all seen).
    """
    return sum(
        len(cluster) for cluster in clusters
        if not target_keys.isdisjoint(cluster)
    )


def overlaps_dead(cluster_keys, dead_clusters, min_fraction):
    """True if at least `min_fraction` of a frontier's cells belong to a
    frontier already given up on (one a visit didn't shrink)."""
    if not cluster_keys:
        return False
    for dead in dead_clusters:
        shared = len(cluster_keys & dead)
        if shared >= min_fraction * len(cluster_keys):
            return True
    return False


def find_frontier_clusters(data, width, height, min_size):
    """Groups of frontier cells: free cells (0) with an unknown (-1) cell
    directly above, below, left or right, grouped 4-connected. Groups with
    fewer than `min_size` cells are dropped as noise.

    Finding the frontier cells is one numpy pass; only those cells are then
    grouped in Python. Checking every map cell in Python took ~0.7 s on a
    40 x 40 m map, on the thread that also runs the obstacle watch.

    The result matches a plain per-cell scan exactly, groups and cell order
    included: groups start in row-major order and grow breadth-first with
    neighbors in the order +x, -x, +y, -y. Callers sample cells by position
    in each group, so the order matters.
    """

    try:  # OccupancyGrid.data is an array('b'): view it without copying
        grid = np.frombuffer(data, dtype=np.int8)
    except TypeError:
        grid = np.asarray(data, dtype=np.int8)
    grid = grid[:width * height].reshape(height, width)

    unknown = grid == -1
    near_unknown = np.zeros_like(unknown)
    near_unknown[:, :-1] |= unknown[:, 1:]   # unknown at x + 1
    near_unknown[:, 1:] |= unknown[:, :-1]   # unknown at x - 1
    near_unknown[:-1, :] |= unknown[1:, :]   # unknown at y + 1
    near_unknown[1:, :] |= unknown[:-1, :]   # unknown at y - 1
    ys, xs = np.nonzero((grid == 0) & near_unknown)   # row-major order

    cells = list(zip(xs.tolist(), ys.tolist()))
    frontier_cells = set(cells)
    visited = set()
    clusters = []

    for start in cells:
        if start in visited:
            continue
        visited.add(start)
        queue = deque([start])
        cluster = []
        while queue:
            cx, cy = queue.popleft()
            cluster.append((cx, cy))
            for neighbor in ((cx + 1, cy), (cx - 1, cy), (cx, cy + 1), (cx, cy - 1)):
                if neighbor in frontier_cells and neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(neighbor)
        if len(cluster) >= min_size:
            clusters.append(cluster)

    return clusters
