"""Scoring frontiers by what a visit would reveal, and tracking frontiers
that a visit did not shrink.

Works on the raw OccupancyGrid data (no ROS), so it can be tested with a
hand-made grid, like open_space.py and obstacle_sensing.py.
"""

import math

# OccupancyGrid values: 0 free, 100 occupied, -1 unknown. At or above this
# counts as a wall (map_server's default occupied threshold of 0.65).
OCCUPIED_THRESHOLD = 65
UNKNOWN = -1


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
    return len(seen)


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
