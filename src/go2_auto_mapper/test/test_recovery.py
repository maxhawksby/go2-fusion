import math

from types import SimpleNamespace

from go2_auto_mapper.recovery import (
    SCAN_MAX_AGE_SEC,
    assess_recovery,
    choose_recovery,
    latch_decision,
    refresh_status,
    scan_covers_move,
    swept_clear,
)
from test_obstacle_sensing import make_grid


def clear_scan():
    return SimpleNamespace(
        angle_min=-math.pi,
        angle_increment=math.pi / 180,
        range_min=0.1,
        range_max=10.0,
        ranges=[5.0] * 361,
    )


def test_rear_blocked_selects_side():
    grid = make_grid(obstacles=[(-1, -0.6, -0.4, 0.4)])
    dx, dy = choose_recovery(grid, 0, 0, 0, 0.3, clear_scan())
    assert dx == 0 and abs(dy) == 0.3


def test_sides_blocked_selects_back():
    grid = make_grid(obstacles=[
        (-0.2, 0.6, 0.35, 0.9),
        (-0.2, 0.6, -0.9, -0.35),
    ])
    dx, dy = choose_recovery(grid, 0, 0, 0, 0.3, clear_scan())
    assert dx == -0.3 and dy == 0


def test_side_translation_checks_long_body():
    grid = make_grid(obstacles=[(-0.45, -0.35, 0.4, 0.5)])
    assert not swept_clear(grid, 0, 0, 0, 0, 0.3)
    assert swept_clear(grid, 0, 0, 0, 0, -0.3)


def test_unknown_and_off_grid_are_unsafe():
    grid = make_grid()
    grid.data[:] = [-1] * len(grid.data)
    assert choose_recovery(grid, 0, 0, 0, 0.3, clear_scan()) is None
    assert not swept_clear(make_grid(), 2.9, 0, 0, 0.3, 0)


def test_boxed_in_stays_stopped():
    grid = make_grid(obstacles=[
        (-1, -0.6, -1, 1),
        (-1, 1, 0.4, 1),
        (-1, 1, -1, -0.4),
    ])
    move, reason = assess_recovery(
        grid, 0, 0, 0, 0.3, clear_scan(), 0.1, SCAN_MAX_AGE_SEC
    )
    assert move is None
    assert 'no safe direction' in reason


def test_stale_scan_is_unsafe():
    grid = make_grid()
    move, reason = assess_recovery(
        grid, 0, 0, 0, 0.3, clear_scan(), 2.0, SCAN_MAX_AGE_SEC
    )
    assert move is None
    assert 'stale' in reason
    move, reason = assess_recovery(
        grid, 0, 0, 0, 0.3, None, None, SCAN_MAX_AGE_SEC
    )
    assert move is None
    assert 'stale' in reason


def test_rotated_footprint_and_new_obstacle():
    assert swept_clear(make_grid(), 0, 0, math.pi / 2, -0.3, 0)
    assert not swept_clear(
        make_grid(obstacles=[(-0.3, 0.3, -0.9, -0.6)]),
        0, 0, math.pi / 2, -0.3, 0,
    )


def test_scan_coverage_requires_measured_free_space():
    scan = clear_scan()
    assert scan_covers_move(scan, -0.3, 0)
    # Rays that look backward (near ±pi). A 0.55 m return is inside the
    # swept tail, so it is not clearance.
    for index, _ in enumerate(scan.ranges):
        angle = scan.angle_min + index * scan.angle_increment
        if abs(math.atan2(math.sin(angle - math.pi), math.cos(angle - math.pi))) < math.radians(15):
            scan.ranges[index] = 0.55
    assert not scan_covers_move(scan, -0.3, 0)
    scan.ranges = [float('inf')] * 361
    assert not scan_covers_move(scan, 0, 0.3)


def _stamps(when):
    return {
        'scan': when,
        'cloud': when,
        'costmap': when,
        'odom_tf': when,
        'map_tf': when,
        'slam_map': when,
    }


def test_refresh_waits_for_observations_after_the_move():
    stop = 5.0
    ready, reason = refresh_status(6.0, stop, 12.0, _stamps(4.0), True)
    assert not ready
    assert reason.startswith('waiting for fresh')
    assert 'scan' in reason and 'slam_map' in reason

    ready, reason = refresh_status(6.0, stop, 12.0, _stamps(5.1), True)
    assert ready
    assert reason == ''


def test_refresh_times_out_and_names_what_is_missing():
    stamps = _stamps(6.0)
    stamps['costmap'] = 1.0
    stamps['cloud'] = None
    ready, reason = refresh_status(20.0, 5.0, 12.0, stamps, True)
    assert not ready
    assert reason.startswith('timed out waiting for fresh')
    assert 'cloud' in reason and 'costmap' in reason
    assert 'scan' not in reason


def test_latch_clears_only_when_the_new_scan_changes_the_block():
    assert latch_decision(True, None) == 'hold'
    assert latch_decision(True, (-0.3, 0.0)) == 'move'
    assert latch_decision(False, None) == 'resume'


def test_short_move_does_not_wait_for_slam_map():
    stamps = _stamps(6.0)
    stamps['slam_map'] = 1.0
    ready, reason = refresh_status(7.0, 5.0, 12.0, stamps, False)
    assert ready
    assert reason == ''
