import math

from types import SimpleNamespace

from go2_auto_mapper.recovery import (
    DRIFT_ACROSS_M,
    DRIFT_YAW_RAD,
    SCAN_MAX_AGE_SEC,
    assess_recovery,
    assess_recovery_steps,
    choose_recovery,
    choose_recovery_mode,
    drift_limits,
    latch_decision,
    recovery_distances,
    refresh_status,
    scan_age,
    stale_sources,
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
    grid = make_grid(obstacles=[(-1.2, -0.8, -0.4, 0.4)])
    dx, dy = choose_recovery(grid, 0, 0, 0, 0.3, clear_scan())
    assert dx == 0 and abs(dy) == 0.3


def test_sides_blocked_selects_back():
    grid = make_grid(obstacles=[
        (-0.2, 0.6, 0.5, 0.9),
        (-0.2, 0.6, -0.9, -0.5),
    ])
    dx, dy = choose_recovery(grid, 0, 0, 0, 0.3, clear_scan())
    assert dx == -0.3 and dy == 0


def test_side_translation_checks_long_body():
    grid = make_grid(obstacles=[(-0.45, -0.35, 0.4, 0.5)])
    assert not swept_clear(grid, 0, 0, 0, 0, 0.3)
    assert swept_clear(grid, 0, 0, 0, 0, -0.3)


def test_obstacle_at_the_nose_does_not_block_backing_away():
    # A wall 0.02 m past the 0.36 m nose, inside the padded body: exactly
    # when recovery runs. Backing away must stay possible, on the costmap
    # and on the scan; sliding sideways along it must not.
    grid = make_grid(obstacles=[(0.38, 0.6, -1, 1)])
    assert swept_clear(grid, 0, 0, 0, -0.3, 0)
    assert not swept_clear(grid, 0, 0, 0, 0.3, 0)
    assert not swept_clear(grid, 0, 0, 0, 0, 0.3)
    scan = clear_scan()
    for index, _ in enumerate(scan.ranges):
        angle = scan.angle_min + index * scan.angle_increment
        if abs(angle) < math.radians(80):
            scan.ranges[index] = 0.38 / math.cos(angle)
    assert scan_covers_move(scan, -0.3, 0)
    assert choose_recovery(grid, 0, 0, 0, 0.3, scan) == (-0.3, 0.0)


def test_wall_alongside_does_not_block_backing_out():
    # A wall 0.3 m beside the body (lethal from y = 0.50), with its inscribed
    # band (99) reaching into the padded footprint. Backing out along it is
    # fine even with the full heading and sideways drift; moving into it is
    # not.
    grid = make_grid(obstacles=[(-2, 2, 0.50, 0.7)])
    for gy in range(grid.info.height):
        for gx in range(grid.info.width):
            y = grid.info.origin.position.y + (gy + 0.5) * grid.info.resolution
            i = gy * grid.info.width + gx
            if 0.30 <= y < 0.50 and grid.data[i] == 0:
                grid.data[i] = 99
    assert swept_clear(grid, 0, 0, 0, -0.3, 0)
    assert not swept_clear(grid, 0, 0, 0, 0, 0.3)
    assert choose_recovery(grid, 0, 0, 0, 0.3, clear_scan()) in ((-0.3, 0.0), (0.0, -0.3))


def test_wall_beside_the_front_half_needs_a_leaving_move():
    # Reviewer's case: a wall 0.01 m from the side of the front half. With
    # the normal drift allowance a small turn at the start swings the nose
    # into it, so no normal move passes. Backing up along it, or sliding
    # straight away from it, never brings the body closer, so those pass as
    # leaving moves (run with the tight CLOSE_DRIFT limits); sliding toward
    # it never does. Before, Oak stayed stopped here for good (go2_sim:
    # wedged against the short wall stub after Nav2 drove it along it).
    grid = make_grid(obstacles=[(0.0, 0.36, 0.21, 0.4)])
    assert not swept_clear(grid, 0, 0, 0, -0.3, 0)
    assert not swept_clear(grid, 0, 0, 0, 0, -0.3)
    assert swept_clear(grid, 0, 0, 0, -0.3, 0, leaving=True)
    assert swept_clear(grid, 0, 0, 0, 0, -0.3, leaving=True)
    assert not swept_clear(grid, 0, 0, 0, 0, 0.3, leaving=True)
    move, leaving = choose_recovery_mode(grid, 0, 0, 0, 0.3, clear_scan())
    assert leaving
    assert move in ((-0.3, 0.0), (0.0, -0.3))
    # 0.15 m away, sliding away from it is a normal move again.
    grid = make_grid(obstacles=[(0.0, 0.36, 0.35, 0.5)])
    assert swept_clear(grid, 0, 0, 0, 0, -0.3)
    assert choose_recovery_mode(grid, 0, 0, 0, 0.3, clear_scan())[1] is False
    scan = clear_scan()
    for index, _ in enumerate(scan.ranges):
        angle = scan.angle_min + index * scan.angle_increment
        if math.radians(30) < angle < math.radians(90):
            scan.ranges[index] = 0.21 / math.sin(angle)
    assert not scan_covers_move(scan, -0.3, 0)
    assert not scan_covers_move(scan, 0, -0.3)
    assert scan_covers_move(scan, -0.3, 0, leaving=True)
    assert scan_covers_move(scan, 0, -0.3, leaving=True)
    assert not scan_covers_move(scan, 0, 0.3, leaving=True)


def test_leaving_move_frees_a_corner_wedged_on_an_angled_wall():
    # go2_sim: a wall down the left side, 0.01 m from the front-left corner
    # and angled 10 degrees in toward it, so it moves away from the body
    # further back. No normal move passes; backing up (or sliding right)
    # only opens the gap.
    slope = math.tan(math.radians(10))
    wall = [
        (along - 0.025, along + 0.025,
         0.21 + slope * (0.35 - along), 0.31 + slope * (0.35 - along))
        for along in (0.35 - 0.05 * k for k in range(20))
    ]
    grid = make_grid(obstacles=wall)
    assert choose_recovery_mode(grid, 0, 0, 0, 0.3, clear_scan())[0] in (
        (-0.3, 0.0), (0.0, -0.3))
    assert choose_recovery_mode(grid, 0, 0, 0, 0.3, clear_scan())[1]
    for move in ((-0.3, 0.0), (0.0, 0.3), (0.0, -0.3)):
        assert not swept_clear(grid, 0, 0, 0, *move)


def test_leaving_move_still_checks_walls_it_closes_in_on():
    # Behind the tail, in the path: backing up runs the tail into it.
    assert not swept_clear(
        make_grid(obstacles=[(-0.65, -0.60, 0.05, 0.30)]),
        0, 0, 0, -0.3, 0, leaving=True,
    )
    # Beside the front, sliding toward it.
    assert not swept_clear(
        make_grid(obstacles=[(0.0, 0.36, 0.30, 0.4)]),
        0, 0, 0, 0, 0.3, leaving=True,
    )
    # Straight behind.
    assert not swept_clear(
        make_grid(obstacles=[(-0.75, -0.70, -0.3, 0.3)]),
        0, 0, 0, -0.3, 0, leaving=True,
    )
    # Inside the body is never left behind.
    assert not swept_clear(
        make_grid(obstacles=[(0.0, 0.1, 0.0, 0.1)]),
        0, 0, 0, -0.3, 0, leaving=True,
    )


def test_leaving_move_slides_out_along_a_wall_cutting_a_rear_corner():
    # go2_sim: a straight wall cutting 0.09 m into the rear-right corner of
    # the footprint at 45 degrees, unknown space behind it, open to the
    # back-left. Backing straight up and sliding right run into the wall,
    # but sliding back-left along it keeps it just as deep.
    cut = 0.09 * math.sqrt(2)
    grid = make_grid()
    for gy in range(grid.info.height):
        for gx in range(grid.info.width):
            x = grid.info.origin.position.x + (gx + 0.5) * grid.info.resolution
            y = grid.info.origin.position.y + (gy + 0.5) * grid.info.resolution
            # Signed distance past the wall line x + y = -(0.45 + 0.2) + cut.
            past = (-(x + y) - (0.65 - cut)) / math.sqrt(2)
            i = gy * grid.info.width + gx
            if 0 <= past < 0.1:
                grid.data[i] = 100
            elif past >= 0.1:
                grid.data[i] = -1
    d = 0.3 / math.sqrt(2)
    assert not swept_clear(grid, 0, 0, 0, -0.3, 0, leaving=True)
    assert not swept_clear(grid, 0, 0, 0, 0, -0.3, leaving=True)
    assert swept_clear(grid, 0, 0, 0, -d, d, leaving=True)
    move, leaving = choose_recovery_mode(grid, 0, 0, 0, 0.3, clear_scan())
    assert leaving


def test_leaving_scan_check_leaves_empty_rays_to_the_costmap():
    # The front-left sector with no returns in go2_sim.
    scan = clear_scan()
    for degrees in range(36, 84):
        scan.ranges[degrees + 180] = float('inf')
    d = 0.3 / math.sqrt(2)
    assert not scan_covers_move(scan, -d, d)
    assert scan_covers_move(scan, -d, d, leaving=True)


def test_leaving_moves_use_tight_drift_limits():
    assert drift_limits(False) == (DRIFT_YAW_RAD, DRIFT_ACROSS_M)
    yaw, across = drift_limits(True)
    assert yaw < DRIFT_YAW_RAD and across < DRIFT_ACROSS_M


def test_drift_envelope_covers_turning_and_sliding():
    # A post the straight body never touches, but that the body reaches if
    # it drifts within the limits tick_recovery allows (DRIFT_ACROSS_M off
    # the line, DRIFT_YAW_RAD of heading). Backing up past it must be
    # refused.
    post = (-0.65, -0.60, 0.33, 0.38)  # beside the tail
    grid = make_grid(obstacles=[post])
    assert not swept_clear(grid, 0, 0, 0, -0.3, 0)
    # Further out than any allowed drift: clear.
    grid = make_grid(obstacles=[(-0.65, -0.60, 0.48, 0.53)])
    assert swept_clear(grid, 0, 0, 0, -0.3, 0)


def test_mid_move_check_is_anchored_at_the_start_pose():
    # A box beside the far end of a 0.6 m back-up, within the drift
    # envelope. Checked from the start pose 0.3 m in, the rest of the move
    # still reaches it, so the move must stop.
    box = make_grid(obstacles=[(-1.0, -0.9, 0.36, 0.42)])
    assert swept_clear(box, 0, 0, 0, -0.3, 0)
    assert not swept_clear(box, 0, 0, 0, -0.6, 0, start=0.3)
    # A wall beside the start, already left behind 0.3 m into the move
    # (with the drift allowance), no longer counts.
    behind = make_grid(obstacles=[(0.3, 0.6, 0.36, 0.42)])
    assert not swept_clear(behind, 0, 0, 0, -0.6, 0)
    assert swept_clear(behind, 0, 0, 0, -0.6, 0, start=0.5)


def test_unknown_and_off_grid_are_unsafe():
    grid = make_grid()
    grid.data[:] = [-1] * len(grid.data)
    assert choose_recovery_mode(
        grid, 0, 0, 0, 0.3, clear_scan(), modes=(False,)
    )[0] is None
    assert not swept_clear(make_grid(), 2.9, 0, 0, 0.3, 0)


def test_boxed_in_stays_stopped():
    grid = make_grid(obstacles=[
        (-1, -0.6, -1, 1),
        (-1, 1, 0.4, 1),
        (-1, 1, -1, -0.4),
    ])
    move, reason, leaving = assess_recovery(
        grid, 0, 0, 0, 0.3, clear_scan(), 0.1, SCAN_MAX_AGE_SEC
    )
    assert move is None and not leaving
    assert 'no safe direction' in reason


def test_recovery_distances_only_shorten_the_requested_move():
    assert recovery_distances(0.3) == (0.3, 0.15, 0.08)
    assert recovery_distances(0.10) == (0.10, 0.08)
    assert recovery_distances(0.05) == (0.05,)


def test_shorter_recovery_is_used_when_full_move_does_not_fit():
    # The padded body starts inside a box with 15-25 cm of extra room. A
    # 30 cm move reaches the walls in every candidate direction, while a
    # 15 cm move stays inside the measured free corridor.
    grid = make_grid(obstacles=[
        (-0.82, -0.70, -1.0, 1.0),
        (-1.0, 1.0, 0.50, 0.65),
        (-1.0, 1.0, -0.65, -0.50),
    ])
    assert choose_recovery_mode(
        grid, 0, 0, 0, 0.3, clear_scan(), modes=(False,)
    )[0] is None
    move, reason, _ = assess_recovery_steps(
        grid, 0, 0, 0, recovery_distances(0.3), clear_scan(),
        0.1, SCAN_MAX_AGE_SEC,
    )
    assert math.isclose(math.hypot(*move), 0.15)
    assert reason == ''


def test_shorter_recovery_keeps_stale_scan_stop():
    move, reason, _ = assess_recovery_steps(
        make_grid(), 0, 0, 0, recovery_distances(0.3), clear_scan(),
        2.0, SCAN_MAX_AGE_SEC,
    )
    assert move is None
    assert 'stale' in reason


def test_stale_scan_is_unsafe():
    grid = make_grid()
    move, reason, _ = assess_recovery(
        grid, 0, 0, 0, 0.3, clear_scan(), 2.0, SCAN_MAX_AGE_SEC
    )
    assert move is None
    assert 'stale' in reason
    move, reason, _ = assess_recovery(
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
        behind = math.atan2(math.sin(angle - math.pi), math.cos(angle - math.pi))
        if abs(behind) < math.radians(15):
            scan.ranges[index] = 0.55
    assert not scan_covers_move(scan, -0.3, 0)
    scan.ranges = [float('inf')] * 361
    assert not scan_covers_move(scan, 0, 0.3)


def test_scan_coverage_wraps_a_full_circle_scan():
    # robot.launch.py's scan: 360 rays from -pi. Straight behind is +pi,
    # which is ray 0, so backing up is measurable.
    scan = clear_scan()
    scan.ranges = [5.0] * 360
    assert scan_covers_move(scan, -0.3, 0)
    assert scan_covers_move(scan, 0, 0.3)


def test_empty_rays_refuse_only_new_space():
    # A few rays with no return. Toward the space the move enters they
    # refuse it; through space the body already stands in (in front of
    # base_link while backing up) the costmap check covers it.
    scan = clear_scan()
    for degrees in (-28, -27, 40):
        scan.ranges[degrees + 180] = float('inf')
    assert scan_covers_move(scan, -0.3, 0)
    scan.ranges[180 + 180] = float('inf')  # straight behind
    assert not scan_covers_move(scan, -0.3, 0)


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


def test_scan_age_uses_acquisition_time_not_just_arrival():
    # Taken 2 s ago, delivered just now: arrival alone would say 0 s.
    age = scan_age(received_mono=100.0, now_mono=100.0,
                   stamp_sec=50.0, ros_now_sec=52.0)
    assert age == 2.0
    assert age > SCAN_MAX_AGE_SEC


def test_scan_age_uses_arrival_when_that_is_older():
    # Stamped 0.1 s ago but this callback ran 1 s after it arrived.
    age = scan_age(received_mono=100.0, now_mono=101.0,
                   stamp_sec=52.9, ros_now_sec=53.0)
    assert abs(age - 1.0) < 1e-9


def test_scan_age_rejects_future_stamps():
    assert scan_age(100.0, 100.0, stamp_sec=55.0, ros_now_sec=53.0) is None
    # Within the tolerance, a tiny clock skew is fine.
    assert scan_age(100.0, 100.0, stamp_sec=53.05, ros_now_sec=53.0) == 0.0


def test_stale_sources_requires_current_data():
    now_ros, now_mono = 100.0, 500.0
    sources = {
        'scan': (499.9, 99.9),     # current
        'cloud': (494.0, 99.95),   # stamped now but arrived 6 s ago: stale
        'costmap': (499.0, 95.0),  # arrived now but taken 5 s ago: stale
        'odom_tf': (None, 99.8),   # TF: arrival unknown, stamp current
        'map_tf': (None, 100.5),   # future-stamped map->odom is normal
        'slam_map': (None, None),  # never received
    }
    assert sorted(stale_sources(now_ros, now_mono, sources, 3.0)) == [
        'cloud', 'costmap', 'slam_map'
    ]
