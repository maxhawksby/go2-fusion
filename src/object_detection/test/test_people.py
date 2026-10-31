import math

from object_detection.people import (
    PeopleTracker,
    find_groups,
    forecast,
    people_dict,
    to_odom,
    velocity,
)


def walk(tracker, start, step, frames, dt=0.2):
    """One person walking in a straight line, one picture every dt."""
    for k in range(frames):
        tracker.update([(start[0] + step[0] * k, start[1] + step[1] * k)], k * dt)
    return (frames - 1) * dt


def test_a_walking_person_keeps_one_id():
    tracker = PeopleTracker()
    walk(tracker, (2.0, 0.0), (0.3, 0.0), 10)
    assert list(tracker.tracks) == [1]


def test_two_people_get_two_ids_and_keep_them():
    tracker = PeopleTracker()
    tracker.update([(0.0, 0.0), (3.0, 0.0)], 0.0)
    tracker.update([(3.1, 0.0), (0.1, 0.0)], 0.2)   # listed in the other order
    assert [round(t.last[1]) for t in tracker.tracks.values()] == [0, 3]


def test_a_far_jump_starts_a_new_track_and_old_ones_expire():
    tracker = PeopleTracker()
    tracker.update([(0.0, 0.0)], 0.0)
    tracker.update([(5.0, 0.0)], 0.2)
    assert sorted(tracker.tracks) == [1, 2]
    tracker.update([], 3.0)
    assert tracker.tracks == {}


def test_velocity_from_a_walk():
    tracker = PeopleTracker()
    now = walk(tracker, (2.0, 0.0), (0.0, 0.2), 10)   # 1 m/s to the left
    vx, vy = velocity(tracker.tracks[1].history, 3.2)
    assert abs(vx) < 1e-9
    assert math.isclose(vy, 1.0)
    assert now == tracker.tracks[1].last[0]


def test_standing_person_has_no_velocity():
    history = [(0.0, 1.0, 1.0), (0.4, 1.05, 0.98), (0.8, 0.97, 1.02)]   # jitter only
    assert velocity(history, 3.2) == (0.0, 0.0)
    assert velocity([(0.0, 1.0, 1.0)], 3.2) == (0.0, 0.0)


def test_forecast_walks_straight():
    path = forecast(0.0, 0.0, 1.0, 0.5, 3, 0.4)
    assert path == [(0.4, 0.2), (0.8, 0.4), (1.2000000000000002, 0.6000000000000001)]


def test_groups_chain_through_neighbors():
    # A and B are 1 m apart, B and C 1.2 m: one group of three; D is alone.
    labels = find_groups([(0, 0), (1, 0), (2.2, 0), (8, 8)], 1.5, 2)
    assert labels == [0, 0, 0, None]


def test_two_separate_groups():
    labels = find_groups([(0, 0), (0.5, 0), (5, 5), (5.5, 5)], 1.5, 2)
    assert labels == [0, 0, 1, 1]
    assert find_groups([], 1.5, 2) == []


def test_people_reports_forecast_and_group():
    tracker = PeopleTracker()
    for k in range(10):
        t = k * 0.2
        tracker.update([(2.0, 0.2 * k), (2.5, 0.2 * k)], t)
    people = tracker.people(1.8)
    assert {p.group for p in people} == {0}
    assert len(people[0].path) == 12
    assert math.isclose(people[0].path[-1][1], people[0].y + 1.0 * 0.4 * 12)


def test_people_missed_for_a_moment_are_still_reported():
    tracker = PeopleTracker()
    tracker.update([(1.0, 0.0)], 0.0)
    tracker.update([], 0.6)
    assert len(tracker.people(0.6)) == 1
    tracker.update([], 1.5)
    assert tracker.people(1.5) == []


def test_to_odom_turns_and_shifts():
    x, y = to_odom(1.0, 0.0, 2.0, 3.0, math.pi / 2)   # robot faces +y
    assert math.isclose(x, 2.0, abs_tol=1e-9)
    assert math.isclose(y, 4.0)


def test_people_dict_shape():
    tracker = PeopleTracker()
    tracker.update([(1.234, 5.678)], 0.0)
    data = people_dict(tracker.people(0.0))
    assert data == {'person_1': {
        'x': 1.23, 'y': 5.68, 'vx': 0.0, 'vy': 0.0, 'group': None,
        'path': [[1.23, 5.68]] * 12,
    }}
