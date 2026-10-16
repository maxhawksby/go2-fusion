import math

from lidar_processor.frame_transform import rotate_point, transform_points


def test_identity_rotation_only_translates():
    points = [(1.0, 2.0, 3.0)]
    moved = transform_points(points, (10.0, -1.0, 0.5), (0.0, 0.0, 0.0, 1.0))
    assert len(moved) == 1
    assert all(abs(a - b) < 1e-9 for a, b in zip(moved[0], (11.0, 1.0, 3.5)))


def test_yaw_90_degrees_maps_x_onto_y():
    half = math.sin(math.pi / 4.0)
    rotated = rotate_point(1.0, 0.0, 0.0, (0.0, 0.0, half, half))
    assert abs(rotated[0]) < 1e-9
    assert abs(rotated[1] - 1.0) < 1e-9
    assert abs(rotated[2]) < 1e-9


def test_loop_closure_shift_moves_every_stored_point():
    # A later map <- odom is 1 m further along x than the one used before.
    # Saving again with that transform must shift the whole cloud, not only
    # points that arrived after the correction.
    odom_points = [(0.0, 0.0, 0.0), (2.0, -1.0, 0.5)]
    before = transform_points(odom_points, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    after = transform_points(odom_points, (1.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    for old, new in zip(before, after):
        assert abs((new[0] - old[0]) - 1.0) < 1e-9
        assert abs(new[1] - old[1]) < 1e-9
        assert abs(new[2] - old[2]) < 1e-9
