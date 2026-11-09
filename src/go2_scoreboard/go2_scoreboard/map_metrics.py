"""Pure occupancy-grid I/O and coverage calculations."""

from dataclasses import dataclass
import math
import os

import numpy as np


FREE = 0
OCCUPIED = 100
UNKNOWN = -1


@dataclass
class Grid:
    data: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float

    @property
    def known_area_m2(self):
        known_cells = np.count_nonzero((self.data >= 0) & (self.data <= 100))
        return float(known_cells) * self.resolution ** 2

    def cell_centers(self):
        rows, cols = self.data.shape
        xs = self.origin_x + (np.arange(cols) + 0.5) * self.resolution
        ys = self.origin_y + (np.arange(rows) + 0.5) * self.resolution
        return np.meshgrid(xs, ys)


def grid_from_values(data, width, height, resolution, origin_x, origin_y):
    values = np.asarray(data, dtype=np.int16).reshape(int(height), int(width))
    return Grid(values, float(resolution), float(origin_x), float(origin_y))


def _read_pgm(path):
    with open(path, 'rb') as stream:
        raw = stream.read()
    tokens = []
    position = 0
    while len(tokens) < 4:
        while raw[position:position + 1].isspace():
            position += 1
        if raw[position:position + 1] == b'#':
            position = raw.index(b'\n', position) + 1
            continue
        end = position
        while raw[end:end + 1] and not raw[end:end + 1].isspace():
            end += 1
        tokens.append(raw[position:end])
        position = end
    while raw[position:position + 1].isspace():
        position += 1
    if tokens[0] != b'P5':
        raise ValueError(f'{path}: only binary PGM (P5) is supported')
    cols, rows, max_value = map(int, tokens[1:])
    if max_value > 255:
        raise ValueError(f'{path}: only 8-bit PGM is supported')
    return np.frombuffer(raw, dtype=np.uint8, count=rows * cols,
                         offset=position).reshape(rows, cols)


def read_map(yaml_path):
    """Read a map_server YAML/PGM pair as an occupancy grid."""
    metadata = {}
    with open(yaml_path, encoding='utf-8') as stream:
        for line in stream:
            if ':' in line and not line.lstrip().startswith('#'):
                key, value = line.split(':', 1)
                metadata[key.strip()] = value.strip()
    image_path = os.path.join(os.path.dirname(yaml_path), metadata['image'])
    resolution = float(metadata['resolution'])
    origin = [float(value) for value in metadata['origin'].strip('[]').split(',')]
    negate = int(metadata.get('negate', 0))
    occupied_threshold = float(metadata.get('occupied_thresh', 0.65))
    free_threshold = float(metadata.get('free_thresh', 0.25))
    image = _read_pgm(image_path)[::-1].astype(np.float64)
    probability = image / 255.0 if negate else (255.0 - image) / 255.0
    values = np.full(image.shape, UNKNOWN, dtype=np.int16)
    values[probability > occupied_threshold] = OCCUPIED
    values[probability < free_threshold] = FREE
    return Grid(values, resolution, origin[0], origin[1])


def write_map(grid, yaml_path):
    """Write an occupancy grid as map_server-compatible YAML and PGM."""
    os.makedirs(os.path.dirname(os.path.abspath(yaml_path)), exist_ok=True)
    pgm_path = os.path.splitext(yaml_path)[0] + '.pgm'
    image = np.full(grid.data.shape, 205, dtype=np.uint8)
    image[(grid.data >= 0) & (grid.data < 25)] = 254
    image[grid.data >= 65] = 0
    rows, cols = image.shape
    with open(pgm_path, 'wb') as stream:
        stream.write(f'P5\n{cols} {rows}\n255\n'.encode())
        stream.write(image[::-1].tobytes())
    with open(yaml_path, 'w', encoding='utf-8') as stream:
        stream.write(
            f'image: {os.path.basename(pgm_path)}\n'
            f'resolution: {grid.resolution}\n'
            f'origin: [{grid.origin_x}, {grid.origin_y}, 0.0]\n'
            # Standard map_server thresholds keep gray 205 as unknown.
            'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n'
        )
    return pgm_path


def _dilate(mask, cells):
    if cells <= 0:
        return mask.copy()
    output = mask.copy()
    rows, cols = mask.shape
    for row_shift in range(-cells, cells + 1):
        for col_shift in range(-cells, cells + 1):
            source = mask[max(0, -row_shift):rows - max(0, row_shift),
                          max(0, -col_shift):cols - max(0, col_shift)]
            output[max(0, row_shift):rows - max(0, -row_shift),
                   max(0, col_shift):cols - max(0, -col_shift)] |= source
    return output


def _interior_mask(grid):
    """Return the enclosed building floor used by the simulator scorer.

    Free space outside a closed building normally touches the grid border.
    Counting the largest component alone can therefore select the outside of
    the building and make an otherwise valid SLAM map score as zero percent.
    Prefer the largest enclosed component and retain the open-world fallback
    used by ``go2_sim.world_tools``.
    """
    free = grid.data == FREE
    rows, cols = free.shape
    visited = np.zeros(free.shape, dtype=bool)
    best = []
    fallback = []
    for start_row, start_col in zip(*np.nonzero(free & ~visited)):
        if visited[start_row, start_col]:
            continue
        stack = [(int(start_row), int(start_col))]
        visited[start_row, start_col] = True
        component = []
        touches_border = False
        while stack:
            row, col = stack.pop()
            component.append((row, col))
            touches_border |= row in (0, rows - 1) or col in (0, cols - 1)
            for next_row, next_col in ((row - 1, col), (row + 1, col),
                                       (row, col - 1), (row, col + 1)):
                if (0 <= next_row < rows and 0 <= next_col < cols
                        and free[next_row, next_col] and not visited[next_row, next_col]):
                    visited[next_row, next_col] = True
                    stack.append((next_row, next_col))
        if len(component) > len(fallback):
            fallback = component
        if not touches_border and len(component) > len(best):
            best = component
    if not best:
        best = fallback
    inside = np.zeros(free.shape, dtype=bool)
    if best:
        points = np.asarray(best)
        inside[points[:, 0], points[:, 1]] = True
    return inside


def score_completeness(ground_truth, slam, start_pose, wall_margin=0.10):
    """Return mapped floor percentage after aligning SLAM to the start pose."""
    scored = _interior_mask(ground_truth)
    scored &= ~_dilate(ground_truth.data >= 65,
                       int(round(wall_margin / ground_truth.resolution)))
    total = int(scored.sum())
    if total == 0:
        return 0.0
    x_grid, y_grid = ground_truth.cell_centers()
    world_x, world_y = x_grid[scored], y_grid[scored]
    start_x, start_y, start_yaw = start_pose
    cosine, sine = math.cos(start_yaw), math.sin(start_yaw)
    map_x = cosine * (world_x - start_x) + sine * (world_y - start_y)
    map_y = -sine * (world_x - start_x) + cosine * (world_y - start_y)
    cols = np.floor((map_x - slam.origin_x) / slam.resolution).astype(int)
    rows = np.floor((map_y - slam.origin_y) / slam.resolution).astype(int)
    height, width = slam.data.shape
    valid = (rows >= 0) & (rows < height) & (cols >= 0) & (cols < width)
    values = np.full(total, UNKNOWN, dtype=np.int16)
    values[valid] = slam.data[rows[valid], cols[valid]]
    known = (values >= 0) & (values <= 100)
    return 100.0 * float(known.sum()) / total
