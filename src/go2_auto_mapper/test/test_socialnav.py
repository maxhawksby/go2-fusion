import base64
import math

from go2_auto_mapper.socialnav import (
    MIN_STEP_SCALE_M,
    History,
    build_request,
    lidar_room,
    step_scale,
    to_map,
    to_robot,
)


def test_history_keeps_one_sample_a_step():
    history = History(length=3, step_sec=1.0)
    assert history.offer(0.0, 0, 0, 0, 'a')
    assert not history.offer(0.5, 0, 0, 0, 'b')
    assert history.offer(1.0, 0, 0, 0, 'c')
    assert history.offer(2.0, 0, 0, 0, 'd')
    assert history.full()
    assert history.offer(3.0, 0, 0, 0, 'e')
    assert [s[4] for s in history.samples] == ['c', 'd', 'e']


def test_robot_frame_is_right_and_forward():
    # Robot at (1, 1) facing +y (north): a point to its north is forward,
    # one to its east is to its right.
    yaw = math.pi / 2
    right, forward = to_robot(1.0, 3.0, 1.0, 1.0, yaw)
    assert math.isclose(right, 0.0, abs_tol=1e-9) and math.isclose(forward, 2.0)
    right, forward = to_robot(2.0, 1.0, 1.0, 1.0, yaw)
    assert math.isclose(right, 1.0) and math.isclose(forward, 0.0, abs_tol=1e-9)


def test_to_map_undoes_to_robot():
    for point in [(3.0, -2.0), (-1.5, 0.5), (0.0, 4.0)]:
        r, f = to_robot(*point, 0.5, -1.0, 0.7)
        x, y = to_map(r, f, 0.5, -1.0, 0.7)
        assert math.isclose(x, point[0], abs_tol=1e-9) and math.isclose(y, point[1], abs_tol=1e-9)


def test_step_scale_has_a_floor_when_standing_still():
    still = [(t, 0.0, 0.0, 0.0, None) for t in range(5)]
    walking = [(t, 0.5 * t, 0.0, 0.0, None) for t in range(5)]
    assert step_scale(still) == MIN_STEP_SCALE_M
    assert math.isclose(step_scale(walking), 0.5)


def test_request_puts_the_robot_at_the_origin():
    samples = [(t, 0.4 * t, 0.0, 0.0, f'f{t}'.encode()) for t in range(5)]
    body = build_request(samples, 5.0, 2.0, encode=lambda frame: frame)
    assert [base64.b64decode(i) for i in body['images']] == [b'f0', b'f1', b'f2', b'f3', b'f4']
    assert body['history'][-1] == [0.0, 0.0]
    assert math.isclose(body['history'][0][1], -1.6)
    assert math.isclose(body['goal'][1], 3.4)      # 3.4 m ahead
    assert math.isclose(body['goal'][0], -2.0)     # 2 m to the left


def test_lidar_room_looks_along_the_waypoint():
    # Readings 1 m ahead and 3 m to the left (angle_min 0, 90 degree steps).
    ranges = [1.0, 3.0, float('inf'), float('inf')]
    step = math.pi / 2
    assert lidar_room(ranges, 0.0, step, 0.0, 0.5, 0.3) is True     # 0.5 m ahead
    assert lidar_room(ranges, 0.0, step, 0.0, 0.9, 0.3) is False    # wall at 1 m
    assert lidar_room(ranges, 0.0, step, -2.0, 0.0, 0.3) is True    # 2 m left
    assert lidar_room(ranges, 0.0, step, 2.0, 0.0, 0.3) is None     # nothing to the right
