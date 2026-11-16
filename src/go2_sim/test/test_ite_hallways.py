"""Keep the hallway benchmark connected and its closed cores unscored."""

from pathlib import Path

import numpy as np

from go2_sim import world_tools as wt


WORLD = Path(__file__).resolve().parents[1] / 'worlds' / 'ite_hallways.sdf'


def test_hallways_and_lobby_form_one_scored_component():
    obstacles, skipped = wt.load_obstacles(WORLD.read_text())
    assert not skipped
    grid = wt.rasterize(obstacles)
    interior = wt.interior_mask(grid)
    # Every free cell within the shell must connect; no hidden room islands.
    x, y = grid.cell_centers()
    floor = (grid.data == wt.FREE) & (x > 0) & (x < 24) & (y > 0) & (y < 24)
    assert np.array_equal(interior, floor)
    for point in [(1.5, 1.5), (22.5, 1.5), (22.5, 22.5), (1.5, 22.5),
                  (1.5, 12), (12, 12), (22.5, 12)]:
        assert interior[grid.world_to_cell(*point)], point
    for point in [(12, 6), (12, 18), (4.5, 10.5), (4.5, 13.5), (-0.5, 12)]:
        assert not interior[grid.world_to_cell(*point)], point


def test_seeded_starts_have_clearance_and_stay_in_hallways():
    obstacles, _ = wt.load_obstacles(WORLD.read_text())
    grid = wt.rasterize(obstacles)
    permitted = wt.interior_mask(grid) & ~wt.dilate(
        grid.data == wt.OCCUPIED, int(np.ceil(0.7 / grid.resolution)))
    for seed in [28000, 28001, 28002, 28003, 28004]:
        pose = wt.sample_start_pose(grid, np.random.default_rng(seed), clearance=0.7)
        assert permitted[grid.world_to_cell(*pose[:2])]
