import math

import numpy as np
import pytest

from go2_scoreboard.lab_measure import (
    Cloud, LabSamples, Scan, box_counts, build_report, measure_repeats, nearest_ahead,
    object_windows, to_base_link,
)
from lab_samples import (
    STOP_COMMANDS, STOP_POSES, TURN_COMMANDS, TURN_GYRO, TURN_POSES, burst, drive,
)


def test_stop_distance_and_time_to_rest_come_from_the_pose_after_the_last_command():
    first, second = measure_repeats(STOP_COMMANDS, STOP_POSES)

    assert first.command_start_s == pytest.approx(2.0)
    assert first.command_end_s == pytest.approx(4.9)
    assert first.distance_after_command_m == pytest.approx(0.4, abs=0.01)
    assert first.rest_s - first.command_end_s == pytest.approx(2.0, abs=0.05)
    assert first.peak_speed_mps == pytest.approx(0.3, abs=0.01)
    assert second.distance_after_command_m == pytest.approx(0.6, abs=0.01)


def test_stop_report_gives_each_repeat_then_median_and_worst():
    report, measured = build_report(
        'stop', LabSamples(commands=STOP_COMMANDS, poses=STOP_POSES))

    assert measured
    assert '| 1 | 2.0 to 4.9 | 0.30 | 0.30 | 1.96 | 0.40 |' in report
    assert '| 2 | 12.0 to 14.9 | 0.30 | 0.30 | 2.96 | 0.60 |' in report
    assert 'Stop distance: median 0.50 m, worst 0.60 m (2 of 2 repeats)' in report


def test_oak_still_moving_when_the_recording_ends_has_no_stop_distance():
    repeat, = measure_repeats(burst(1.0, 30, linear=0.3), drive(4.2, [(1.0, 9.0, 0.3)]))

    assert repeat.rest_s is None
    assert repeat.distance_after_command_m is None


def test_latency_separates_when_oak_moved_from_when_odometry_reported_it():
    repeat, = measure_repeats(TURN_COMMANDS, TURN_POSES, TURN_GYRO)

    assert repeat.gyro_motion_start_s - repeat.command_start_s == pytest.approx(0.5, abs=0.03)
    assert repeat.motion_start_s - repeat.command_start_s == pytest.approx(2.0, abs=0.1)
    assert repeat.peak_turn_rate_radps == pytest.approx(0.8, abs=0.02)


def test_turn_report_shows_the_odometry_lag():
    report, _ = build_report(
        'turn', LabSamples(commands=TURN_COMMANDS, poses=TURN_POSES, gyro=TURN_GYRO))

    assert '| 1 | 1.0 | 0.50 | 2.08 | 1.58 | 0.80 | 0.80 |' in report
    assert 'the odometry reports late' in report


def test_missing_imu_still_reports_the_odometry_delay_and_says_what_is_unknown():
    samples = LabSamples(commands=TURN_COMMANDS, poses=TURN_POSES)
    repeat, = measure_repeats(samples.commands, samples.poses)
    report, measured = build_report('turn', samples)

    assert repeat.gyro_motion_start_s is None
    assert repeat.motion_start_s - repeat.command_start_s == pytest.approx(2.0, abs=0.1)
    assert measured
    assert '| 1 | 1.0 | - | 2.08 | - | 0.80 | 0.80 |' in report
    assert 'cannot say whether the delay is Oak or the odometry' in report


@pytest.mark.parametrize('kind', ['stop', 'turn', 'object'])
def test_empty_recording_reports_nothing_measured(kind):
    report, measured = build_report(kind, LabSamples())

    assert not measured
    assert 'found' in report


def test_commands_without_odometry_give_a_repeat_with_nothing_measured():
    repeat, = measure_repeats(TURN_COMMANDS, [])

    assert repeat.command_start_s == pytest.approx(1.0)
    assert repeat.motion_start_s is None
    assert repeat.rest_s is None


def _scan(t, returns, beams=360):
    """A full-circle scan with a return of `range` at each `bearing` in degrees."""
    ranges = [math.inf] * beams
    for bearing, distance in returns.items():
        ranges[bearing % beams] = distance
    return Scan(t, 0.0, math.tau / beams, tuple(ranges))


def test_nearest_scan_return_ignores_what_is_beside_or_behind_oak():
    scan = _scan(0.0, {0: 0.5, 90: 0.3, 180: 0.2, 10: 1.0})

    assert nearest_ahead(scan) == pytest.approx(0.5)
    assert nearest_ahead(_scan(0.0, {90: 0.3})) is None


def test_cloud_points_in_the_box_ahead_are_counted_by_height():
    points = [
        (0.5, 0.0, -0.30),
        (0.5, 0.1, -0.10), (1.0, -0.2, -0.20),
        (0.8, 0.0, 0.20),
        (0.1, 0.0, -0.10), (1.5, 0.0, -0.10), (0.5, 0.4, -0.10),
    ]

    assert box_counts(points) == (1, 2, 1)
    assert box_counts(np.empty((0, 3))) == (0, 0, 0)


def test_odom_cloud_is_moved_into_oaks_own_frame():
    quarter_turn_left = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))

    moved = to_base_link([(1.0, 3.0, 0.1)], (1.0, 2.0, 0.3), quarter_turn_left)

    assert moved[0] == pytest.approx((1.0, 0.0, -0.2))


def test_object_windows_follow_the_backpack_as_it_moves_closer():
    backpack = np.array([(0.5, 0.0, -0.1)] * 40)
    windows = object_windows(
        scans=[_scan(1.0, {}), _scan(6.0, {0: 0.5}), _scan(7.0, {0: 0.52})],
        clouds=[Cloud(1.0, np.empty((0, 3))), Cloud(6.0, backpack)])

    assert [w.start_s for w in windows] == [0.0, 5.0]
    assert windows[0].nearest_m is None
    assert windows[0].mean_points_by_band == (0, 0, 0)
    assert windows[1].nearest_m == pytest.approx(0.5)
    assert windows[1].median_nearest_m == pytest.approx(0.51)
    assert windows[1].mean_points_by_band == (0, 40, 0)
