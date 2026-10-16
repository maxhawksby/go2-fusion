import math
from types import SimpleNamespace

from go2_auto_mapper.obstacle_sensing import (
    LETHAL_COST,
    ROBOT_FRONT_M,
    angle_difference,
    cost_at,
    free_distance_ahead,
    straight_line_clear,
)

RESOLUTION = 0.05


def make_grid(width_m=6.0, height_m=6.0, obstacles=()):
    """Empty costmap centered on (0, 0); `obstacles` are (x_min, x_max,
    y_min, y_max) world boxes filled with lethal cells."""
    width = int(width_m / RESOLUTION)
    height = int(height_m / RESOLUTION)
    origin_x, origin_y = -width_m / 2.0, -height_m / 2.0
    data = [0] * (width * height)
    for x_min, x_max, y_min, y_max in obstacles:
        for gy in range(height):
            for gx in range(width):
                x = origin_x + (gx + 0.5) * RESOLUTION
                y = origin_y + (gy + 0.5) * RESOLUTION
                if x_min <= x <= x_max and y_min <= y <= y_max:
                    data[gy * width + gx] = LETHAL_COST
    info = SimpleNamespace(
        resolution=RESOLUTION, width=width, height=height,
        origin=SimpleNamespace(position=SimpleNamespace(x=origin_x, y=origin_y)),
    )
    return SimpleNamespace(info=info, data=data)


# A wall across the robot's path, 1.0 m ahead of base_link (along +x).
WALL_AHEAD = (1.0, 1.1, -1.0, 1.0)


def test_cost_at_returns_none_off_grid():
    grid = make_grid()
    assert cost_at(grid, 10.0, 0.0) is None
    assert cost_at(grid, 0.0, 0.0) == 0


def test_free_distance_ahead_is_none_in_open_space():
    assert free_distance_ahead(make_grid(), 0.0, 0.0, 0.0, 1.0) is None


def test_free_distance_ahead_measures_gap_from_the_nose():
    grid = make_grid(obstacles=[WALL_AHEAD])
    gap = free_distance_ahead(grid, 0.0, 0.0, 0.0, 1.0)
    # The wall starts 1.0 m from base_link; the nose is ROBOT_FRONT_M ahead.
    assert gap is not None
    assert abs(gap - (1.0 - ROBOT_FRONT_M)) <= 2 * RESOLUTION


def test_free_distance_ahead_ignores_walls_beyond_look_ahead():
    grid = make_grid(obstacles=[WALL_AHEAD])
    assert free_distance_ahead(grid, 0.0, 0.0, 0.0, 0.3) is None


def test_free_distance_ahead_ignores_walls_behind_and_beside():
    grid = make_grid(obstacles=[
        (-1.1, -1.0, -1.0, 1.0),   # behind
        (0.0, 1.5, 0.6, 0.7),      # to the left, outside the body strip
    ])
    assert free_distance_ahead(grid, 0.0, 0.0, 0.0, 1.0) is None


def test_free_distance_ahead_follows_heading():
    grid = make_grid(obstacles=[WALL_AHEAD])
    # Facing +y, the wall at +x is beside the robot, not ahead.
    assert free_distance_ahead(grid, 0.0, 0.0, math.pi / 2, 1.0) is None


def test_straight_line_clear_detects_wall_on_the_way():
    grid = make_grid(obstacles=[WALL_AHEAD])
    assert not straight_line_clear(grid, 0.0, 0.0, 0.0, 2.0)
    assert straight_line_clear(grid, 0.0, 0.0, 0.0, 0.5)
    assert straight_line_clear(grid, 0.0, 0.0, math.pi, 2.0)


def test_angle_difference_wraps_around():
    assert abs(angle_difference(math.radians(170), math.radians(-170))
               - math.radians(20)) < 1e-9
    assert angle_difference(0.0, 0.0) == 0.0
