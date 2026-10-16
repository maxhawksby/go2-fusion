"""Finds the nearest roomy spot on the SLAM map, for getting unstuck.

"Roomy" means known-free and at least `clearance` cells from anything that
isn't known-free (walls, and unknown space that could hide them). The search
walks outward from the robot through known-free cells only, so the spot it
returns is connected to the robot on the map, not just close as the crow
flies. Works on the raw OccupancyGrid data (no ROS), so it can be tested with
a hand-made grid.
"""

from collections import deque

# OccupancyGrid values: 0 free, 100 occupied, -1 unknown. At or above this
# counts as a wall (map_server's default occupied threshold of 0.65).
OCCUPIED_THRESHOLD = 65

# 8-connected: diagonal steps count as one, so distances are Chebyshev,
# which never exceed the true (Euclidean) distance. A spot that's
# `clearance` steps from every wall is therefore at least that far in reality.
NEIGHBORS_8 = (
    (1, 0), (-1, 0), (0, 1), (0, -1),
    (1, 1), (1, -1), (-1, 1), (-1, -1),
)


def is_free(value):
    return 0 <= value < OCCUPIED_THRESHOLD


def find_open_space(data, width, height, start, clearance, max_radius,
                    min_move=0):
    """Nearest cell (by path through free space from `start`) that is at
    least `clearance` cells from any non-free cell and at least `min_move`
    cells from `start`, searching up to `max_radius` cells away.

    `data` is the OccupancyGrid's row-major list, `start` an (x, y) cell.
    Returns (x, y, clearance_in_cells), or None if nothing within reach is
    roomier than where the robot already is. If no cell reaches `clearance`,
    the roomiest one found is returned instead.
    """
    sx, sy = start
    reach = max_radius + clearance
    x0, x1 = max(0, sx - reach), min(width - 1, sx + reach)
    y0, y1 = max(0, sy - reach), min(height - 1, sy + reach)

    clearances = _clearance_map(data, width, x0, y0, x1, y1)

    def clearance_at(x, y):
        return clearances[(y - y0) * (x1 - x0 + 1) + (x - x0)]

    best = None
    best_clearance = clearance_at(sx, sy) if x0 <= sx <= x1 and y0 <= sy <= y1 else 0
    seen = {(sx, sy)}
    queue = deque([(sx, sy)])

    while queue:
        x, y = queue.popleft()
        if max(abs(x - sx), abs(y - sy)) >= min_move:
            room = clearance_at(x, y)
            if room >= clearance:
                return x, y, room
            if room > best_clearance:
                best, best_clearance = (x, y, room), room

        for dx, dy in NEIGHBORS_8:
            nx, ny = x + dx, y + dy
            if (nx, ny) in seen:
                continue
            if not (x0 <= nx <= x1 and y0 <= ny <= y1):
                continue
            if max(abs(nx - sx), abs(ny - sy)) > max_radius:
                continue
            if not is_free(data[ny * width + nx]):
                continue
            seen.add((nx, ny))
            queue.append((nx, ny))

    return best


def _clearance_map(data, width, x0, y0, x1, y1):
    """Steps from each cell in the window to the nearest non-free cell
    (8-connected breadth-first search from all of them at once)."""
    win_w = x1 - x0 + 1
    win_h = y1 - y0 + 1
    unset = win_w + win_h  # larger than any distance inside the window
    dist = [unset] * (win_w * win_h)
    queue = deque()

    for y in range(y0, y1 + 1):
        for x in range(x0, x1 + 1):
            if not is_free(data[y * width + x]):
                dist[(y - y0) * win_w + (x - x0)] = 0
                queue.append((x, y))

    while queue:
        x, y = queue.popleft()
        here = dist[(y - y0) * win_w + (x - x0)]
        for dx, dy in NEIGHBORS_8:
            nx, ny = x + dx, y + dy
            if not (x0 <= nx <= x1 and y0 <= ny <= y1):
                continue
            i = (ny - y0) * win_w + (nx - x0)
            if dist[i] > here + 1:
                dist[i] = here + 1
                queue.append((nx, ny))

    return dist
