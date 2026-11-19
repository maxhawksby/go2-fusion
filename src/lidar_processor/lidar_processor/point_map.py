"""Accumulated 3D point map that follows changes in the environment.

Each /point_cloud2 from the Go2 is the robot's fresh local map of its
surroundings. Inside the area that cloud covers, it replaces what the
accumulated map held; areas the robot can't see right now keep their old
points. Without this the map only ever grew: furniture that moved, people
who walked by, and a whole previous room all stayed in it for good.

Coverage is tracked on a 2D grid of `cell_size` cells: a cell counts as seen
when the new cloud has any point in it. The floor is in the cloud all around
the robot, so the cells near it are refreshed even where an obstacle is gone.

Each cell holds a numpy array, and a cloud (~100k points) is grouped by cell
in a few array operations: per-point Python took ~2.3 s a cloud, longer than
the gap between clouds. No ROS imports, so tests can use it directly.
"""

from collections import OrderedDict

import numpy as np

_EMPTY = np.empty((0, 3))


def _rows(points):
    """One opaque value per point, so whole rows can be compared at once."""
    points = np.ascontiguousarray(points)
    return points.view(np.dtype((np.void, points.dtype.itemsize * 3))).ravel()


class CellPointMap:
    """Points grouped by 2D cell, cells ordered by when they were last seen.

    Not thread-safe; the caller holds a lock.
    """

    def __init__(self, cell_size, max_points):
        if cell_size <= 0:
            raise ValueError('cell_size must be positive')
        self.cell_size = cell_size
        self.max_points = max_points
        # cell (ix, iy) -> (k, 3) array of points, oldest-seen cell first
        self._cells = OrderedDict()
        self._count = 0

    def update(self, points):
        """Replace the contents of every cell `points` covers with `points`.

        Points are rounded to the millimeter and de-duplicated (as before).
        Returns how many old points were dropped because a fresh view of
        their cell no longer had them.
        """

        new = np.asarray(points, dtype=np.float64).reshape(-1, 3)
        new = new[np.isfinite(new).all(axis=1)]
        if not len(new):
            return 0
        new = np.unique(np.round(new, 3), axis=0)

        cells = np.floor(new[:, :2] / self.cell_size).astype(np.int64)
        order = np.lexsort((cells[:, 1], cells[:, 0]))
        new, cells = new[order], cells[order]
        boundary = np.flatnonzero(np.any(np.diff(cells, axis=0) != 0, axis=1)) + 1
        starts = np.concatenate(([0], boundary))
        ends = np.concatenate((boundary, [len(new)]))

        dropped = 0
        for start, end in zip(starts.tolist(), ends.tolist()):
            cell = (int(cells[start, 0]), int(cells[start, 1]))
            cell_points = new[start:end]
            old = self._cells.pop(cell, None)
            if old is not None:
                self._count -= len(old)
                dropped += int(np.count_nonzero(
                    ~np.isin(_rows(old), _rows(cell_points))
                ))
            self._cells[cell] = cell_points       # now the most recently seen
            self._count += len(cell_points)

        # Over the limit: drop the cells seen longest ago first.
        while self._count > self.max_points and self._cells:
            _, old = self._cells.popitem(last=False)
            self._count -= len(old)

        return dropped

    def points(self):
        """All points as an (N, 3) array."""
        if not self._cells:
            return _EMPTY.copy()
        return np.concatenate(list(self._cells.values()))

    def clear(self):
        count = self._count
        self._cells.clear()
        self._count = 0
        return count

    def __len__(self):
        return self._count
