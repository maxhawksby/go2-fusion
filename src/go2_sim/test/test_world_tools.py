"""Unit tests for go2_sim.world_tools (no ROS or Gazebo needed)."""

import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from go2_sim import world_tools as wt  # noqa: E402

PKG = os.path.join(os.path.dirname(__file__), '..')
WORLD = os.path.join(PKG, 'worlds', 'placeholder_rooms.sdf')
TEMPLATE = os.path.join(PKG, 'models', 'oak', 'model.sdf.template')


def _world_text():
    with open(WORLD) as f:
        return f.read()


def test_placeholder_world_obstacles():
    obstacles, skipped = wt.load_obstacles(_world_text())
    names = {o['name'] for o in obstacles}
    assert 'outer_south' in names and 'sofa' in names and 'pillar' in names
    assert 'ground_plane' not in names
    assert skipped == []


def test_ground_truth_interior_area():
    obstacles, _ = wt.load_obstacles(_world_text())
    grid = wt.rasterize(obstacles)
    inside = wt.interior_mask(grid)
    area = inside.sum() * grid.resolution ** 2
    # 10 x 8 m flat minus walls and furniture: roughly 70 m2.
    assert 60 < area < 78, area
    # A point in each room is inside; a point outside the building is not.
    for x, y in [(2.5, 2.0), (2.5, 5.5), (7.5, 1.0), (7.5, 7.0)]:
        r, c = grid.world_to_cell(x, y)
        assert inside[r, c], (x, y)
    r, c = grid.world_to_cell(-0.6, -0.6)
    assert not inside[r, c]


def test_low_obstacle_outside_band_is_ignored():
    text = _world_text().replace(
        '</world>',
        '<model name="rug"><static>true</static><pose>2 2 0.01 0 0 0</pose>'
        '<link name="l"><collision name="c"><geometry><box><size>1 1 0.02</size>'
        '</box></geometry></collision></link></model></world>')
    obstacles, _ = wt.load_obstacles(text)
    assert 'rug' not in {o['name'] for o in obstacles}


def test_map_round_trip(tmp_path):
    obstacles, _ = wt.load_obstacles(_world_text())
    grid = wt.rasterize(obstacles)
    path = str(tmp_path / 'gt.yaml')
    wt.write_map(grid, path)
    back = wt.read_map(path)
    assert back.data.shape == grid.data.shape
    assert np.array_equal(back.data, grid.data)
    assert math.isclose(back.origin_x, grid.origin_x)


def test_start_poses_are_seeded_and_clear():
    obstacles, _ = wt.load_obstacles(_world_text())
    grid = wt.rasterize(obstacles)
    a = [wt.sample_start_pose(grid, np.random.default_rng(7)) for _ in range(2)]
    assert a[0] == a[1]
    rng = np.random.default_rng(1)
    for _ in range(50):
        x, y, _yaw = wt.sample_start_pose(grid, rng, clearance=0.7)
        r, c = grid.world_to_cell(x, y)
        k = int(0.7 / grid.resolution)
        assert (grid.data[r - k:r + k + 1, c - k:c + k + 1] == wt.FREE).all()


def test_completeness_full_and_empty_and_rotated():
    obstacles, _ = wt.load_obstacles(_world_text())
    gt = wt.rasterize(obstacles)
    start = (2.5, 2.0, 0.7)
    # A perfect SLAM map: the ground truth expressed in the map frame.
    x0, y0, yaw0 = start
    res = 0.05
    rows, cols = 400, 400
    slam = wt.Grid(np.full((rows, cols), 255, np.uint8), res, -10.0, -10.0)
    mx, my = slam.cell_centers()
    c, s = math.cos(yaw0), math.sin(yaw0)
    wx = x0 + c * mx - s * my
    wy = y0 + s * mx + c * my
    gr = np.floor((wy - gt.origin_y) / gt.resolution).astype(int)
    gc = np.floor((wx - gt.origin_x) / gt.resolution).astype(int)
    ok = (gr >= 0) & (gr < gt.shape[0]) & (gc >= 0) & (gc < gt.shape[1])
    # Only what a robot inside could see: the building, not the outdoors.
    seen = wt.interior_mask(gt) | (gt.data == wt.OCCUPIED)
    ok[ok] &= seen[gr[ok], gc[ok]]
    slam.data[ok] = gt.data[gr[ok], gc[ok]]
    assert wt.score_completeness(gt, slam, start) > 99.0
    # Wrong start pose: much worse.
    assert wt.score_completeness(gt, slam, (7.5, 7.0, 0.0)) < 80.0
    # Half the map unknown: about half.
    half = wt.Grid(slam.data.copy(), res, -10.0, -10.0)
    half.data[:, :200] = 255
    pct = wt.score_completeness(gt, half, start)
    assert 5 < pct < 95
    empty = wt.Grid(np.full((rows, cols), 255, np.uint8), res, -10.0, -10.0)
    assert wt.score_completeness(gt, empty, start) == 0.0


def test_compose_world_places_robot_and_keeps_plugins():
    with open(TEMPLATE) as f:
        robot = wt.render_robot(f.read(), lidar_rate=2.0)
    composed = wt.compose_world(_world_text(), robot, (1.0, 2.0, 0.32, 0.5))
    root = ET.fromstring(composed)
    world = root.find('world')
    oak = [m for m in world.findall('model') if m.get('name') == 'oak']
    assert len(oak) == 1
    assert oak[0].find('pose').text.split()[:2] == ['1.0000', '2.0000']
    sensor = oak[0].find('link').find("sensor[@name='lidar']")
    assert float(sensor.find('update_rate').text) == 2.0
    # The world already has every required system: none added twice.
    names = [p.get('filename') for p in world.findall('plugin')]
    assert len(names) == len(set(names))


def test_camera_only_when_asked_for():
    with open(TEMPLATE) as f:
        template = f.read()
    plain = ET.fromstring(wt.render_robot(template))
    assert plain.find("link/sensor[@name='front_camera']") is None
    camera = ET.fromstring(wt.render_robot(template, camera_rate=5.0)).find(
        "link/sensor[@name='front_camera']")
    assert float(camera.find('update_rate').text) == 5.0
    assert camera.find('topic').text == '/sim/camera'


def test_compose_world_adds_missing_plugins():
    with open(TEMPLATE) as f:
        robot = wt.render_robot(f.read())
    bare = '<sdf version="1.8"><world name="w"></world></sdf>'
    composed = wt.compose_world(bare, robot, (0, 0, 0.32, 0))
    names = [p.get('filename') for p in ET.fromstring(composed).find('world').findall('plugin')]
    assert 'ignition-gazebo-sensors-system' in names
    assert 'ignition-gazebo-contact-system' in names
