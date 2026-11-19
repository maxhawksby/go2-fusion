"""The simulated Oak moves, sees and meets obstacles the way Oak did on 2026-11-16 (#129).

Plain pytest and numpy, no ROS: sim_driver.py loads with test_sim_parity's fake ROS.
"""

import math
import sys
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from go2_sim import world_tools as wt  # noqa: E402
from test_sim_parity import load_sim_driver  # noqa: E402

SIM_YAML = Path(__file__).resolve().parents[1] / 'config' / 'sim.yaml'
DIALS = yaml.safe_load(SIM_YAML.read_text())['go2_sim_driver']['ros__parameters']
drv = load_sim_driver()


def respond(vx, wz, u=0.0):
    return drv.oak_response(vx, 0.0, wz, u, DIALS['min_walk_speed'], DIALS['min_turn_rate'],
                            DIALS['sure_turn_rate'], DIALS['unsure_turn_success'],
                            DIALS['turn_gain'])


def test_standing_oak_ignores_slow_turns():
    assert respond(0.0, 0.2) == (0.0, 0.0, 0.0)
    assert respond(0.0, -0.29) == (0.0, 0.0, 0.0)


def test_slow_walk_commands_do_not_move_oak():
    assert respond(0.1, 0.0) == (0.0, 0.0, 0.0)


def test_middle_turns_work_only_sometimes():
    assert respond(0.0, 0.5, u=0.1)[2] != 0.0
    assert respond(0.0, 0.5, u=0.9)[2] == 0.0


def test_fast_turns_always_work_at_oaks_gain():
    vx, _, wz = respond(0.0, 1.0, u=0.99)
    assert vx == 0.0
    assert math.isclose(wz, 0.8)


def test_walking_oak_makes_small_turns():
    vx, _, wz = respond(0.3, 0.1, u=0.99)
    assert vx == 0.3
    assert math.isclose(wz, 0.08)


def test_blind_sectors_drop_only_their_directions():
    pts = np.array([[1.0, 0.0], [-1.0, 0.0], [0.0, 1.0]])
    front = int(36 * 0.5)    # azimuth 0 sits at the middle sector
    blind = drv.blind_points(pts, np.array([front]), 36)
    assert blind.tolist() == [True, False, False]


def grid():
    obstacles, _ = wt.load_obstacles(
        (Path(__file__).resolve().parents[1] / 'worlds' / 'placeholder_rooms.sdf').read_text())
    return wt.rasterize(obstacles)


def test_floor_objects_land_on_open_floor_away_from_the_start():
    g = grid()
    start = wt.sample_start_pose(g, np.random.default_rng(1))
    spots = wt.place_floor_objects(g, np.random.default_rng(2), start, count=3)
    assert len(spots) == 3
    inside = wt.interior_mask(g)
    for x, y, _ in spots:
        assert math.hypot(x - start[0], y - start[1]) >= 1.5
        assert inside[g.world_to_cell(x, y)]
    assert spots == wt.place_floor_objects(g, np.random.default_rng(2), start, count=3)


def test_floor_objects_are_solid_and_left_out_of_the_ground_truth():
    world = (Path(__file__).resolve().parents[1] / 'worlds' / 'placeholder_rooms.sdf').read_text()
    with_objects = wt.add_floor_objects(world, [(3.0, 2.0, 0.0)])
    assert '<collision name="collision">' in with_objects.split('floor_object_0')[1]
    before, _ = wt.load_obstacles(world)
    after, _ = wt.load_obstacles(with_objects)
    assert len(after) == len(before)
