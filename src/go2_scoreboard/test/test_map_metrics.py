import numpy as np

from go2_scoreboard.map_metrics import Grid, read_map, score_completeness, write_map


def test_known_area_and_map_round_trip(tmp_path):
    grid = Grid(np.array([[0, -1], [100, 0]], dtype=np.int16), 0.5, -1.0, 2.0)
    assert grid.known_area_m2 == 0.75
    path = tmp_path / 'map.yaml'
    write_map(grid, str(path))
    restored = read_map(str(path))
    np.testing.assert_array_equal(restored.data, grid.data)
    assert restored.resolution == grid.resolution
    assert restored.origin_x == grid.origin_x
    assert restored.origin_y == grid.origin_y


def test_completeness_is_nullable_by_caller_and_geometrically_scored():
    truth = Grid(np.zeros((4, 4), dtype=np.int16), 1.0, 0.0, 0.0)
    slam = Grid(np.full((4, 4), -1, dtype=np.int16), 1.0, 0.0, 0.0)
    slam.data[:2, :] = 0
    assert score_completeness(truth, slam, (0.0, 0.0, 0.0), wall_margin=0.0) == 50.0


def test_completeness_ignores_larger_free_space_outside_building():
    truth_data = np.full((8, 8), 0, dtype=np.int16)
    truth_data[1:7, 1] = 100
    truth_data[1:7, 6] = 100
    truth_data[1, 1:7] = 100
    truth_data[6, 1:7] = 100
    truth = Grid(truth_data, 1.0, 0.0, 0.0)
    slam = Grid(np.full((8, 8), -1, dtype=np.int16), 1.0, 0.0, 0.0)
    slam.data[2:6, 2:4] = 0

    assert score_completeness(
        truth, slam, (0.0, 0.0, 0.0), wall_margin=0.0) == 50.0
