"""World helpers for the Oak simulator, with no ROS imports so they unit-test anywhere.

- compose_world: puts the Oak model and the Gazebo systems it needs into any world.
- ground truth: turns a world's box and cylinder obstacles into an occupancy
  grid (or loads one shipped next to the world), in map_server's PGM + YAML format.
- sample_start_pose and score_completeness: what the trial runner needs.
"""

import math
import os
import re
import string
import xml.etree.ElementTree as ET
from collections import deque

import numpy as np

FREE = 0
OCCUPIED = 100

# Height band (m above the floor) that the 2D /scan sees: robot.launch.py keeps
# points from -0.1 to +0.5 m around base_link, which sits about 0.32 m up.
DEFAULT_BAND = (0.22, 0.82)

# Gazebo Fortress systems the simulator needs. Added to a world that lacks them.
REQUIRED_WORLD_PLUGINS = [
    ('ignition-gazebo-physics-system', 'ignition::gazebo::systems::Physics', ''),
    ('ignition-gazebo-sensors-system', 'ignition::gazebo::systems::Sensors',
     '<render_engine>ogre2</render_engine>'),
    ('ignition-gazebo-scene-broadcaster-system',
     'ignition::gazebo::systems::SceneBroadcaster', ''),
    ('ignition-gazebo-user-commands-system',
     'ignition::gazebo::systems::UserCommands', ''),
    ('ignition-gazebo-contact-system', 'ignition::gazebo::systems::Contact', ''),
]


# ----------------------------------------------------------------------------
# World composition
# ----------------------------------------------------------------------------

def render_robot(template_text, base_height=0.32, lidar_rate=5.0,
                 lidar_range=12.0, lidar_samples=720, lidar_noise=0.01, camera_rate=0.0):
    """Fill the $fields of the Oak model template (models/oak/model.sdf.template).

    base_height is how high base_link rides above the floor; the feet slab,
    bumper and legs are sized from it. camera_rate > 0 adds the front camera
    at that many pictures a second; 0 leaves it out, since rendering it costs
    CPU on laptops without a GPU.
    """
    bumper_bottom = 0.08                      # floor clearance of the bumper
    bumper_top = base_height + 0.08           # top of the body
    values = dict(
        base_height=base_height,
        feet_z=-(base_height - 0.02),
        bumper_z=(bumper_bottom + bumper_top) / 2 - base_height,
        bumper_h=bumper_top - bumper_bottom,
        leg_len=base_height - 0.02,
        leg_z=-(base_height - 0.02) / 2 - 0.02,
        lidar_rate=lidar_rate,
        lidar_range=lidar_range,
        lidar_samples=int(lidar_samples),
        lidar_noise=lidar_noise,
        camera=CAMERA_SENSOR.format(rate=camera_rate) if camera_rate > 0 else '',
    )
    return string.Template(template_text).substitute(
        {k: (f'{v:.4f}' if isinstance(v, float) else str(v))
         for k, v in values.items()})


# Oak's front camera: in the head, looking straight ahead (go2.urdf's
# front_camera). 1280x720 like the real one would be slow to render on a CPU,
# so a quarter of that; the VLM shrinks pictures to 448 wide anyway. The
# horizontal field of view is a guess. TODO(owner): measure the real camera's.
CAMERA_SENSOR = """
    <sensor name="front_camera" type="camera">
      <pose>0.31 0 0.05 0 0 0</pose>
      <topic>/sim/camera</topic>
      <update_rate>{rate}</update_rate>
      <always_on>1</always_on>
      <visualize>false</visualize>
      <camera>
        <horizontal_fov>1.5708</horizontal_fov>
        <image><width>640</width><height>360</height><format>R8G8B8</format></image>
        <clip><near>0.05</near><far>30</far></clip>
        <visibility_mask>4294967293</visibility_mask>
      </camera>
    </sensor>"""


def world_name(world_text):
    root = ET.fromstring(world_text)
    world = root.find('world')
    if world is None:
        raise ValueError('No <world> element in the world file')
    return world.get('name', 'default')


def compose_world(world_text, robot_model_text, pose):
    """Return the world SDF with Oak placed at pose (x, y, z, yaw) and the
    required systems added. robot_model_text is a bare <model> element."""
    x, y, z, yaw = pose
    if '<world' not in world_text:
        raise ValueError('No <world> element in the world file')
    plugins = ''
    for filename, name, body in REQUIRED_WORLD_PLUGINS:
        short = filename.replace('ignition-gazebo-', '').replace('-system', '')
        # Accept either Fortress (ignition-gazebo-*) or newer (gz-sim-*) names.
        if re.search(r'filename\s*=\s*"[^"]*' + re.escape(short), world_text):
            continue
        plugins += f'    <plugin filename="{filename}" name="{name}">{body}</plugin>\n'
    model = robot_model_text.strip()
    model = re.sub(r'<\?xml[^>]*\?>', '', model).strip()
    # Place the model: replace (or add) its top-level <pose>.
    pose_tag = f'<pose>{x:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.5f}</pose>'
    model = re.sub(r'(<model\b[^>]*>)', r'\1\n    ' + pose_tag, model, count=1)
    insert = plugins + '    ' + model + '\n'
    idx = world_text.rfind('</world>')
    return world_text[:idx] + insert + world_text[idx:]


# ----------------------------------------------------------------------------
# Reading obstacles out of an SDF world
# ----------------------------------------------------------------------------

def _pose(elem):
    """(x, y, z, yaw) of an element's <pose> child; roll and pitch are ignored."""
    p = elem.find('pose') if elem is not None else None
    if p is None or not (p.text or '').strip():
        return (0.0, 0.0, 0.0, 0.0)
    v = [float(t) for t in p.text.split()]
    v += [0.0] * (6 - len(v))
    return (v[0], v[1], v[2], v[5])


def _compose(a, b):
    ax, ay, az, ayaw = a
    bx, by, bz, byaw = b
    c, s = math.cos(ayaw), math.sin(ayaw)
    return (ax + c * bx - s * by, ay + s * bx + c * by, az + bz, ayaw + byaw)


def load_obstacles(world_text, band=DEFAULT_BAND, skip_models=('oak',)):
    """Obstacles in the world whose height overlaps band.

    Returns (obstacles, skipped). Each obstacle is a dict with kind 'box'
    (x, y, yaw, sx, sy) or 'cylinder' (x, y, r). skipped lists geometry that
    could not be read (meshes, includes), which a shipped ground truth must cover.
    """
    root = ET.fromstring(world_text)
    world = root.find('world')
    obstacles, skipped = [], []

    def walk_model(model, parent_pose):
        name = model.get('name', '')
        if name in skip_models:
            return
        mpose = _compose(parent_pose, _pose(model))
        for nested in model.findall('model'):
            walk_model(nested, mpose)
        for link in model.findall('link'):
            lpose = _compose(mpose, _pose(link))
            for col in link.findall('collision'):
                cpose = _compose(lpose, _pose(col))
                geom = col.find('geometry')
                if geom is None:
                    continue
                box, cyl = geom.find('box'), geom.find('cylinder')
                if box is not None:
                    sx, sy, sz = [float(t) for t in box.find('size').text.split()]
                    zmin, zmax = cpose[2] - sz / 2, cpose[2] + sz / 2
                    if zmax < band[0] or zmin > band[1]:
                        continue
                    obstacles.append(dict(kind='box', x=cpose[0], y=cpose[1],
                                          yaw=cpose[3], sx=sx, sy=sy, name=name))
                elif cyl is not None:
                    r = float(cyl.find('radius').text)
                    h = float(cyl.find('length').text)
                    zmin, zmax = cpose[2] - h / 2, cpose[2] + h / 2
                    if zmax < band[0] or zmin > band[1]:
                        continue
                    obstacles.append(dict(kind='cylinder', x=cpose[0], y=cpose[1],
                                          r=r, name=name))
                elif geom.find('plane') is not None:
                    continue
                else:
                    skipped.append(f'{name}/{col.get("name", "?")}')

    for model in world.findall('model'):
        walk_model(model, (0.0, 0.0, 0.0, 0.0))
    for inc in world.findall('include'):
        uri = inc.find('uri')
        skipped.append(f'include {uri.text if uri is not None else "?"}')
    return obstacles, skipped


# ----------------------------------------------------------------------------
# Ground-truth grid
# ----------------------------------------------------------------------------

class Grid:
    """Occupancy grid: data[row, col], row 0 at origin_y (bottom), like ROS."""

    def __init__(self, data, resolution, origin_x, origin_y):
        self.data = data
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y

    @property
    def shape(self):
        return self.data.shape

    def cell_centers(self):
        rows, cols = self.data.shape
        xs = self.origin_x + (np.arange(cols) + 0.5) * self.resolution
        ys = self.origin_y + (np.arange(rows) + 0.5) * self.resolution
        return np.meshgrid(xs, ys)

    def world_to_cell(self, x, y):
        return (int(math.floor((y - self.origin_y) / self.resolution)),
                int(math.floor((x - self.origin_x) / self.resolution)))


def rasterize(obstacles, resolution=0.05, margin=1.0):
    if not obstacles:
        raise ValueError('The world has no box or cylinder obstacles to build a '
                         'ground truth from; ship a <world>.gt.yaml next to it')
    xs, ys = [], []
    for o in obstacles:
        ext = math.hypot(o['sx'], o['sy']) / 2 if o['kind'] == 'box' else o['r']
        xs += [o['x'] - ext, o['x'] + ext]
        ys += [o['y'] - ext, o['y'] + ext]
    ox = math.floor((min(xs) - margin) / resolution) * resolution
    oy = math.floor((min(ys) - margin) / resolution) * resolution
    cols = int(math.ceil((max(xs) + margin - ox) / resolution))
    rows = int(math.ceil((max(ys) + margin - oy) / resolution))
    grid = Grid(np.zeros((rows, cols), dtype=np.uint8), resolution, ox, oy)
    gx, gy = grid.cell_centers()
    occ = np.zeros((rows, cols), dtype=bool)
    for o in obstacles:
        dx, dy = gx - o['x'], gy - o['y']
        if o['kind'] == 'box':
            c, s = math.cos(o['yaw']), math.sin(o['yaw'])
            u = c * dx + s * dy
            v = -s * dx + c * dy
            # Half a cell of slack so thin walls never fall between cell centres.
            h = resolution / 2
            occ |= (np.abs(u) <= o['sx'] / 2 + h) & (np.abs(v) <= o['sy'] / 2 + h)
        else:
            occ |= dx * dx + dy * dy <= (o['r'] + resolution / 2) ** 2
    grid.data[occ] = OCCUPIED
    return grid


def _components(free):
    """Label 4-connected components of a boolean mask. Returns (labels, count)."""
    rows, cols = free.shape
    labels = np.zeros(free.shape, dtype=np.int32)
    count = 0
    for r0 in range(rows):
        for c0 in range(cols):
            if not free[r0, c0] or labels[r0, c0]:
                continue
            count += 1
            labels[r0, c0] = count
            queue = deque([(r0, c0)])
            while queue:
                r, c = queue.popleft()
                for nr, nc in ((r + 1, c), (r - 1, c), (r, c + 1), (r, c - 1)):
                    if 0 <= nr < rows and 0 <= nc < cols and free[nr, nc] \
                            and not labels[nr, nc]:
                        labels[nr, nc] = count
                        queue.append((nr, nc))
    return labels, count


def interior_mask(grid):
    """Free cells of the biggest free area that does not touch the grid border:
    the inside of the building. Open worlds (no closed area) use every free cell."""
    free = grid.data == FREE
    labels, count = _components(free)
    border = set(np.unique(np.concatenate([
        labels[0, :], labels[-1, :], labels[:, 0], labels[:, -1]])).tolist())
    best, best_size = None, 0
    for lab in range(1, count + 1):
        if lab in border:
            continue
        size = int((labels == lab).sum())
        if size > best_size:
            best, best_size = lab, size
    if best is None:
        return free
    return labels == best


def dilate(mask, cells):
    """Grow a boolean mask by a square of +-cells."""
    out = mask.copy()
    rows, cols = mask.shape
    for dr in range(-cells, cells + 1):
        for dc in range(-cells, cells + 1):
            src = mask[max(0, -dr):rows - max(0, dr), max(0, -dc):cols - max(0, dc)]
            out[max(0, dr):rows - max(0, -dr), max(0, dc):cols - max(0, -dc)] |= src
    return out


def scored_mask(grid, wall_margin=0.10):
    """Cells that count toward completeness: the building's free floor, minus a
    thin strip along walls and furniture where cell edges make the result noisy."""
    inside = interior_mask(grid)
    near = dilate(grid.data == OCCUPIED, int(round(wall_margin / grid.resolution)))
    return inside & ~near


# ----------------------------------------------------------------------------
# map_server PGM + YAML
# ----------------------------------------------------------------------------

def write_map(grid, yaml_path):
    """Save as map_server files: <name>.yaml and <name>.pgm next to it."""
    pgm_path = os.path.splitext(yaml_path)[0] + '.pgm'
    rows, cols = grid.data.shape
    img = np.full((rows, cols), 205, dtype=np.uint8)
    img[grid.data == FREE] = 254
    img[grid.data == OCCUPIED] = 0
    img = img[::-1]  # PGM row 0 is the top of the map
    with open(pgm_path, 'wb') as f:
        f.write(f'P5\n{cols} {rows}\n255\n'.encode())
        f.write(img.tobytes())
    with open(yaml_path, 'w') as f:
        f.write(f'image: {os.path.basename(pgm_path)}\n'
                f'resolution: {grid.resolution}\n'
                f'origin: [{grid.origin_x}, {grid.origin_y}, 0.0]\n'
                'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    return pgm_path


def _read_pgm(path):
    with open(path, 'rb') as f:
        data = f.read()
    tokens, pos = [], 0
    while len(tokens) < 4:
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':
            pos = data.index(b'\n', pos) + 1
            continue
        end = pos
        while not data[end:end + 1].isspace():
            end += 1
        tokens.append(data[pos:end])
        pos = end
    pos += 1
    if tokens[0] != b'P5':
        raise ValueError(f'{path}: only binary PGM (P5) is supported')
    cols, rows, maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
    if maxval > 255:
        raise ValueError(f'{path}: only 8-bit PGM is supported')
    return np.frombuffer(data, dtype=np.uint8, count=rows * cols,
                         offset=pos).reshape(rows, cols)


def read_map(yaml_path):
    """Load a map_server YAML (trinary interpretation) into a Grid."""
    meta = {}
    with open(yaml_path) as f:
        for line in f:
            if ':' in line and not line.lstrip().startswith('#'):
                k, v = line.split(':', 1)
                meta[k.strip()] = v.strip()
    image = os.path.join(os.path.dirname(yaml_path), meta['image'])
    resolution = float(meta['resolution'])
    origin = [float(t) for t in meta['origin'].strip('[]').split(',')]
    negate = int(meta.get('negate', 0))
    occ_t = float(meta.get('occupied_thresh', 0.65))
    free_t = float(meta.get('free_thresh', 0.25))
    img = _read_pgm(image)[::-1].astype(np.float64)
    p = img / 255.0 if negate else (255.0 - img) / 255.0
    data = np.full(img.shape, 255, dtype=np.uint8)  # unknown
    data[p > occ_t] = OCCUPIED
    data[p < free_t] = FREE
    return Grid(data, resolution, origin[0], origin[1])


def ground_truth_for_world(world_path, cache_dir, band=DEFAULT_BAND):
    """The ground-truth grid of a world.

    Uses <world>.gt.yaml next to the world file when it exists (for worlds
    with meshes, e.g. a scanned lab), else builds one from the world's boxes
    and cylinders and caches it in cache_dir. Returns (grid, yaml_path, notes).
    """
    stem = os.path.splitext(world_path)[0]
    shipped = stem + '.gt.yaml'
    if os.path.isfile(shipped):
        return read_map(shipped), shipped, []
    with open(world_path) as f:
        text = f.read()
    obstacles, skipped = load_obstacles(text, band)
    grid = rasterize(obstacles)
    os.makedirs(cache_dir, exist_ok=True)
    out = os.path.join(cache_dir, os.path.basename(stem) + '.gt.yaml')
    write_map(grid, out)
    notes = [f'ground truth ignores unreadable geometry: {s}' for s in skipped]
    return grid, out, notes


# ----------------------------------------------------------------------------
# Trials: start poses and scoring
# ----------------------------------------------------------------------------

def sample_start_pose(grid, rng, clearance=0.7, tries=20000):
    """A random (x, y, yaw) inside the building with clearance metres of free
    floor around it. rng is a numpy Generator, so a seed gives the same pose."""
    inside = interior_mask(grid)
    blocked = dilate(~inside, int(math.ceil(clearance / grid.resolution)))
    rows, cols = np.nonzero(inside & ~blocked)
    if len(rows) == 0:
        raise ValueError(f'No spot with {clearance} m clearance in the world')
    i = int(rng.integers(len(rows)))
    x = grid.origin_x + (cols[i] + 0.5) * grid.resolution
    y = grid.origin_y + (rows[i] + 0.5) * grid.resolution
    yaw = float(rng.uniform(-math.pi, math.pi))
    return float(x), float(y), yaw


def score_completeness(gt, slam, start_pose, wall_margin=0.10):
    """Percent of the building's floor the SLAM map has seen (free or wall).

    gt: ground-truth Grid in world coordinates. slam: Grid of the SLAM map,
    in the map frame, with 255 or -1 meaning unknown (any value other than
    FREE/OCCUPIED-ish counts as unknown). start_pose: Oak's world (x, y, yaw)
    at start, which is where SLAM's map frame begins.
    """
    mask = scored_mask(gt, wall_margin)
    total = int(mask.sum())
    if total == 0:
        return 0.0
    gx, gy = gt.cell_centers()
    wx, wy = gx[mask], gy[mask]
    x0, y0, yaw0 = start_pose
    c, s = math.cos(yaw0), math.sin(yaw0)
    mx = c * (wx - x0) + s * (wy - y0)
    my = -s * (wx - x0) + c * (wy - y0)
    col = np.floor((mx - slam.origin_x) / slam.resolution).astype(int)
    row = np.floor((my - slam.origin_y) / slam.resolution).astype(int)
    rows, cols = slam.data.shape
    inside = (row >= 0) & (row < rows) & (col >= 0) & (col < cols)
    vals = np.full(total, -1, dtype=np.int16)
    vals[inside] = slam.data[row[inside], col[inside]].astype(np.int16)
    known = (vals >= 0) & (vals <= 100)
    return 100.0 * float(known.sum()) / total
