import math
import time

from go2_auto_mapper.socialnav_follow import (
    ANSWER_TIMEOUT_SEC,
    Answer,
    SocialNavClient,
    body_offset,
    goal_reached,
    has_lidar_room,
    next_step,
    waypoints_to_map,
)


def close(a, b, tol=1e-9):
    return all(abs(x - y) <= tol for x, y in zip(a, b))


def test_waypoints_turn_into_map_points_from_the_pose_they_were_asked_at():
    # Facing +y (yaw 90 deg) at (1, 2): 1 m forward is (1, 3); 1 m right is (2, 2).
    points = waypoints_to_map([[0.0, 1.0], [1.0, 0.0]], (1.0, 2.0, math.pi / 2))
    assert close(points[0], (1.0, 3.0))
    assert close(points[1], (2.0, 2.0))


def test_body_offset_is_forward_and_left_of_the_robot():
    # Facing +y at the origin: a point at (-1, 0) is 1 m to the left.
    assert close(body_offset(-1.0, 0.0, 0.0, 0.0, math.pi / 2), (0.0, 1.0))
    assert close(body_offset(0.0, 2.0, 0.0, 0.0, math.pi / 2), (2.0, 0.0))


def test_next_step_strafes_straight_to_the_first_waypoint():
    remaining, move = next_step([(0.3, 0.4), (1.0, 1.0)], (0.0, 0.0, 0.0))
    assert close(move, (0.3, 0.4))          # x forward, y left: a diagonal slide
    assert remaining == ((0.3, 0.4), (1.0, 1.0))


def test_next_step_caps_a_step_at_half_a_meter():
    _, move = next_step([(2.0, 0.0)], (0.0, 0.0, 0.0))
    assert close(move, (0.5, 0.0))


def test_next_step_skips_waypoints_already_reached():
    remaining, move = next_step([(0.05, 0.0), (0.0, 0.4)], (0.0, 0.0, 0.0))
    assert remaining == ((0.0, 0.4),)
    assert close(move, (0.0, 0.4))


def test_next_step_with_every_waypoint_reached_has_no_move():
    assert next_step([(0.02, 0.0)], (0.0, 0.0, 0.0)) == ((), None)
    assert next_step([], (0.0, 0.0, 0.0)) == ((), None)


def test_goal_counts_as_reached_within_half_a_meter():
    assert goal_reached((1.0, 1.0), (1.3, 1.3))
    assert goal_reached((1.0, 1.0), (1.5, 1.0))        # exactly 0.5 m counts
    assert not goal_reached((1.0, 1.0), (1.6, 1.0))


def test_lidar_room_none_means_nothing_in_range_that_way():
    assert has_lidar_room(True)
    assert has_lidar_room(None)
    assert not has_lidar_room(False)


def wait_for(client, seconds=2.0):
    deadline = time.monotonic() + seconds
    while client.busy() and time.monotonic() < deadline:
        time.sleep(0.01)
    return client.take_result()


SAMPLES = [(float(i), 0.1 * i, 0.0, 0.0, b'jpeg') for i in range(5)]


def test_client_returns_the_waypoints_with_the_pose_they_belong_to():
    sent = {}

    def fake_post(url, body, timeout):
        sent.update(body=body, timeout=timeout)
        return {'waypoints': [[0.0, 0.3]] * 5, 'seconds': 0.4}

    client = SocialNavClient('http://x', encode=lambda f: f, post=fake_post)
    assert client.submit(SAMPLES, 3.0, 0.0)
    answer = wait_for(client)
    assert isinstance(answer, Answer)
    assert answer.waypoints == [[0.0, 0.3]] * 5
    assert close(answer.pose, (0.4, 0.0, 0.0))     # the newest sample
    assert sent['timeout'] == ANSWER_TIMEOUT_SEC
    assert len(sent['body']['images']) == 5
    assert client.take_result() is None             # handed out once


def test_client_reports_a_down_server_instead_of_raising():
    def failing_post(url, body, timeout):
        raise OSError('connection refused')

    client = SocialNavClient('http://x', encode=lambda f: f, post=failing_post)
    client.submit(SAMPLES, 3.0, 0.0)
    answer = wait_for(client)
    assert answer.waypoints is None
    assert 'connection refused' in answer.reason


def test_client_rejects_a_reply_without_five_waypoints():
    client = SocialNavClient('http://x', encode=lambda f: f,
                             post=lambda url, body, timeout: {'waypoints': [[0.0, 0.3]]})
    client.submit(SAMPLES, 3.0, 0.0)
    assert wait_for(client).waypoints is None


def test_client_runs_one_request_at_a_time():
    def slow_post(url, body, timeout):
        time.sleep(0.2)
        return {'waypoints': [[0.0, 0.3]] * 5}

    client = SocialNavClient('http://x', encode=lambda f: f, post=slow_post)
    assert client.submit(SAMPLES, 3.0, 0.0)
    assert not client.submit(SAMPLES, 3.0, 0.0)
    wait_for(client)
