"""ROS-dependent tests of the explorer's spot memory; run in the Humble container."""

from concurrent.futures import Future
import math
import time
from types import MethodType, SimpleNamespace

import pytest

from go2_auto_mapper.explorer import FrontierExplorer
from go2_auto_mapper.spot_memory import Spot, SpotMemory

# A 10 m x 10 m map of free space at 0.1 m per cell, origin at (0, 0).
RESOLUTION = 0.1
SIZE = 100

# The explorer methods these tests run for real; Nav2 and the map are faked.
REAL_METHODS = (
    'start_escape', 'closest_remembered_spot', 'spot_still_open', 'request_escape',
    'record_goal_failure',
    'record_escape_interrupted', 'navigation_result_callback',
    '_is_current_navigation_goal', 'remember_scan_spot', 'cancel_for_obstacle',
    'handle_stalled_goal', '_exploration_loop_impl',
)


def free_map():
    return SimpleNamespace(
        info=SimpleNamespace(
            resolution=RESOLUTION, width=SIZE, height=SIZE,
            origin=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0))),
        data=[0] * (SIZE * SIZE))


def explorer_stub(position=(5.0, 5.0), use_spot_memory=True, blacklisted=()):
    messages, goals, blacklist, cancels = [], [], [], []
    node = SimpleNamespace(
        map=free_map(), use_spot_memory=use_spot_memory,
        spot_memory=SpotMemory(size=10, min_clearance=0.8),
        escape_spot=None, escape_skip_memory=False, escape_pending=False,
        last_escape_time=None, open_space_clearance=0.8, goal_is_escape=False,
        goal_frontier=None, consecutive_failures=0, stuck_failure_count=3,
        latest_scan=None, scan_yaw=2.0 * math.pi, scan_age=0.1,
        goal_active=False, goal_sent_time=None, goal_timeout_sec=60.0,
        current_goal=None, current_goal_handle=None, goal_sequence=0,
        active_goal_sequence=0, shutting_down=False,
        goal_cancelled_for_scan=False, goal_cancelled_for_obstacle=False,
        scan_after_goal=False, scan_pending=False, backup_pending=False,
        recovery_move=None, recovery_fault=None, refresh_after=None,
        maneuver_active=False, blocked_time=None, blocked_heading=None,
        messages=messages, goals=goals, blacklist=blacklist, cancels=cancels,
    )
    logger = SimpleNamespace(info=lambda m, **k: messages.append(m),
                             warn=lambda m, **k: messages.append(m),
                             error=lambda m, **k: messages.append(m))
    node.get_logger = lambda: logger
    node.get_robot_position = lambda: position
    node.world_to_grid = lambda x, y: (x / RESOLUTION, y / RESOLUTION)
    node.grid_to_world = lambda gx, gy: ((gx + 0.5) * RESOLUTION, (gy + 0.5) * RESOLUTION)
    node.is_goal_blacklisted = lambda x, y: (x, y) in blacklisted
    node.blacklist_current_goal = lambda: blacklist.append(node.current_goal)
    node._scan_age = lambda: node.scan_age
    for name in ('emit_goal_outcome', 'emit_event', 'log_goal_ended', 'log_event'):
        setattr(node, name, lambda *args: None)

    def send_goal(x, y, is_escape=False):
        goals.append((x, y, is_escape))
        node.goal_is_escape = is_escape
        node.goal_active = True
        node.goal_sequence += 1
        node.active_goal_sequence = node.goal_sequence
        node.current_goal = (x, y)
        node.current_goal_handle = SimpleNamespace(
            cancel_goal_async=lambda: cancels.append((x, y)))
        node.goal_sent_time = 0.0
        node.goal_cancelled_for_obstacle = False
        return True

    node.send_goal = send_goal
    for name in REAL_METHODS:
        setattr(node, name, MethodType(getattr(FrontierExplorer, name), node))
    return node


def finish_goal(node, status):
    """Nav2 reports the current goal ended with `status` (4 reached, 5 canceled, 6 aborted)."""
    future = Future()
    future.set_result(SimpleNamespace(status=status))
    node.navigation_result_callback(future, node.active_goal_sequence)


def scan(ranges):
    return SimpleNamespace(ranges=ranges, range_min=0.1, range_max=10.0,
                           angle_min=-math.pi, angle_increment=2 * math.pi / len(ranges))


# ---- choosing where to escape to ----

def test_escape_goes_to_the_closest_remembered_spot_first():
    node = explorer_stub()
    node.spot_memory.remember(2.0, 2.0, clearance=1.0, stamp=1.0)
    node.spot_memory.remember(7.0, 7.0, clearance=1.0, stamp=2.0)
    node.start_escape()
    assert node.goals == [(7.0, 7.0, True)]
    assert node.escape_spot == Spot(7.0, 7.0, 1.0, 2.0)


def test_escape_skips_a_remembered_spot_that_is_now_blocked_on_the_map():
    node = explorer_stub()
    node.spot_memory.remember(2.0, 2.0, clearance=1.0, stamp=1.0)
    node.spot_memory.remember(7.0, 7.0, clearance=1.0, stamp=2.0)
    node.map.data[70 * SIZE + 72] = 100   # a new obstacle 0.2 m from (7, 7)
    node.start_escape()
    assert node.goals == [(2.0, 2.0, True)]


def test_escape_skips_a_blacklisted_spot():
    node = explorer_stub(blacklisted={(7.0, 7.0)})
    node.spot_memory.remember(7.0, 7.0, clearance=1.0, stamp=2.0)
    node.start_escape()
    assert node.escape_spot is None
    assert node.goals and node.goals[0][:2] != (7.0, 7.0)   # nearest open space instead


def test_escape_without_memory_uses_the_nearest_open_space():
    node = explorer_stub(use_spot_memory=False)
    node.spot_memory.remember(7.0, 7.0, clearance=1.0, stamp=2.0)
    node.start_escape()
    assert node.escape_spot is None
    assert node.goals and node.goals[0][:2] != (7.0, 7.0)


def test_escape_prefers_the_closest_spot_over_the_most_recent():
    node = explorer_stub()
    node.spot_memory.remember(6.0, 5.0, clearance=1.0, stamp=1.0)   # 1 m away, older
    node.spot_memory.remember(9.0, 9.0, clearance=1.0, stamp=2.0)   # 4 m away, newest
    node.start_escape()
    assert node.goals == [(6.0, 5.0, True)]


def test_escape_measures_closest_by_walking_around_walls():
    node = explorer_stub()
    for y in range(0, 95):                  # a wall at x = 5.5 m, open only near the top
        node.map.data[y * SIZE + 55] = 100
    node.spot_memory.remember(7.0, 5.0, clearance=1.0, stamp=2.0)   # 2 m away, behind the wall
    node.spot_memory.remember(2.0, 5.0, clearance=1.0, stamp=1.0)   # 3 m away, same side
    node.start_escape()
    assert node.goals == [(2.0, 5.0, True)]


def test_escape_ignores_remembered_spots_it_cannot_walk_to():
    node = explorer_stub()
    for y in range(SIZE):                   # a wall cutting the map in two
        node.map.data[y * SIZE + 55] = 100
    node.spot_memory.remember(7.0, 5.0, clearance=1.0, stamp=2.0)
    node.start_escape()
    assert node.escape_spot is None
    assert node.goals and node.goals[0][:2] != (7.0, 5.0)   # nearest open space instead


# ---- how a trip back ends, through the real Nav2 result handler ----

def test_an_obstacle_interrupting_the_trip_back_keeps_the_spot():
    node = explorer_stub()
    node.spot_memory.remember(7.0, 7.0, clearance=1.0, stamp=2.0)
    node.start_escape()
    node.cancel_for_obstacle()
    assert node.cancels == [(7.0, 7.0)]
    assert node.backup_pending
    finish_goal(node, 5)
    assert node.spot_memory.spots == (Spot(7.0, 7.0, 1.0, 2.0),)   # first strike: kept
    assert node.blacklist == []
    assert not node.escape_pending and not node.escape_skip_memory   # no instant fallback
    assert not node.goal_active and node.escape_spot is None


def test_a_second_obstacle_on_the_way_to_the_same_spot_forgets_it():
    node = explorer_stub()
    node.spot_memory.remember(7.0, 7.0, clearance=1.0, stamp=2.0)
    for _ in range(2):
        node.start_escape()
        node.cancel_for_obstacle()
        finish_goal(node, 5)
    assert node.spot_memory.spots == ()
    assert node.blacklist == []


def test_an_obstacle_interrupting_the_open_space_escape_does_not_blacklist_it():
    node = explorer_stub()
    node.start_escape()                     # no memory: nearest open space
    node.cancel_for_obstacle()
    finish_goal(node, 5)
    assert node.blacklist == []
    assert not node.escape_pending


def test_an_obstacle_cancelling_an_exploration_goal_still_counts_as_a_failure():
    node = explorer_stub()
    node.send_goal(8.0, 8.0)
    node.cancel_for_obstacle()
    finish_goal(node, 5)
    assert node.blacklist == [(8.0, 8.0)]
    assert node.consecutive_failures == 1


def test_nav2_failing_the_trip_back_forgets_the_spot_and_falls_back_once():
    node = explorer_stub()
    node.spot_memory.remember(7.0, 7.0, clearance=1.0, stamp=2.0)
    node.start_escape()
    finish_goal(node, 6)                    # aborted by Nav2: no obstacle cancel
    assert node.spot_memory.spots == ()
    assert node.escape_pending and node.escape_skip_memory
    node.start_escape()
    assert node.goals[1][:2] != (7.0, 7.0) and node.goals[1][2]
    assert not node.escape_skip_memory      # only that one fallback skips memory


def test_reaching_the_spot_clears_it_and_scans():
    node = explorer_stub()
    node.spot_memory.remember(7.0, 7.0, clearance=1.0, stamp=2.0)
    node.start_escape()
    finish_goal(node, 4)
    assert node.escape_spot is None and node.scan_pending
    assert node.spot_memory.spots == (Spot(7.0, 7.0, 1.0, 2.0),)


def test_failing_the_open_space_fallback_just_carries_on_exploring():
    node = explorer_stub()
    node.goal_is_escape = True
    node.record_goal_failure()
    assert not node.escape_pending
    assert any('carrying on exploring' in m for m in node.messages)


def test_an_obstacle_on_a_long_trip_back_does_not_queue_another_escape():
    node = explorer_stub()
    node.spot_memory.remember(7.0, 7.0, clearance=1.0, stamp=2.0)
    node.start_escape()
    node.last_escape_time = time.monotonic() - 1000.0   # trip outlasted the cooldown
    node.cancel_for_obstacle()
    finish_goal(node, 5)
    assert node.backup_pending and not node.escape_pending
    assert node.spot_memory.spots == (Spot(7.0, 7.0, 1.0, 2.0),)


def test_a_requested_scan_cancel_wins_over_a_late_obstacle_cancel():
    node = explorer_stub()
    node.spot_memory.remember(7.0, 7.0, clearance=1.0, stamp=2.0)
    node.start_escape()
    node.goal_cancelled_for_scan = True     # Scan button pressed first
    node.cancel_for_obstacle()
    finish_goal(node, 5)
    assert node.spot_memory.strikes == {}   # paused for the scan, no strike
    assert node.spot_memory.spots == (Spot(7.0, 7.0, 1.0, 2.0),)
    assert node.blacklist == []


def test_an_obstacle_cancel_nav2_never_answers_still_only_interrupts():
    node = explorer_stub()
    node.spot_memory.remember(7.0, 7.0, clearance=1.0, stamp=2.0)
    node.start_escape()
    node.cancel_for_obstacle()
    node.goal_sent_time = -1000.0           # no result before the timeout
    assert node.handle_stalled_goal()
    assert node.spot_memory.spots == (Spot(7.0, 7.0, 1.0, 2.0),)
    assert node.blacklist == [] and not node.escape_pending
    assert not node.goal_active and not node.goal_cancelled_for_obstacle


@pytest.mark.parametrize('obstacle_too', [False, True])
def test_a_requested_scan_cancel_nav2_never_answers_is_a_pause(obstacle_too):
    node = explorer_stub()
    node.spot_memory.remember(7.0, 7.0, clearance=1.0, stamp=2.0)
    node.start_escape()
    node.goal_cancelled_for_scan = True
    if obstacle_too:
        node.cancel_for_obstacle()
    node.goal_sent_time = -1000.0
    assert node.handle_stalled_goal()
    assert node.spot_memory.strikes == {}
    assert node.spot_memory.spots == (Spot(7.0, 7.0, 1.0, 2.0),)
    assert node.blacklist == [] and not node.escape_pending
    assert not node.goal_cancelled_for_scan and not node.goal_cancelled_for_obstacle


# ---- a trip back that times out, through the real exploration loop ----

def test_a_timed_out_trip_back_runs_the_fallback_before_any_frontier_goal():
    node = explorer_stub()
    node.spot_memory.remember(7.0, 7.0, clearance=1.0, stamp=2.0)
    node.start_escape()
    node.goal_sent_time = -1000.0           # long past goal_timeout_sec

    def no_frontier_yet():
        raise AssertionError('picked a frontier before the fallback')

    node.choose_frontier = no_frontier_yet
    node.get_robot_position = lambda: (5.0, 5.0)
    node._exploration_loop_impl()           # timeout tick: stops there
    assert node.cancels == [(7.0, 7.0)]
    assert node.escape_pending and not node.goal_active
    node._exploration_loop_impl()           # next tick: the fallback escape
    assert len(node.goals) == 2 and node.goals[1][2]
    assert node.goals[1][:2] != (7.0, 7.0)


# ---- recording a spot after the Scan turn ----

def test_a_finished_scan_with_room_is_remembered():
    node = explorer_stub(position=(3.0, 4.0))
    node.latest_scan = scan([1.5, 2.0, 0.9] * 120)
    node.remember_scan_spot()
    assert [(s.x, s.y, s.clearance) for s in node.spot_memory.spots] == [(3.0, 4.0, 0.9)]


def test_a_finished_scan_in_a_tight_spot_is_not_remembered():
    node = explorer_stub()
    node.latest_scan = scan([1.5, 0.4] * 180)
    node.remember_scan_spot()
    assert node.spot_memory.spots == ()


def test_an_old_scan_is_not_remembered():
    node = explorer_stub()
    node.latest_scan = scan([2.0] * 360)
    node.scan_age = 2.0
    node.remember_scan_spot()
    assert node.spot_memory.spots == ()


def test_a_patchy_scan_is_not_remembered():
    node = explorer_stub()
    node.latest_scan = scan([2.0] * 180 + [math.nan] * 180)
    node.remember_scan_spot()
    assert node.spot_memory.spots == ()


def test_a_partial_scan_turn_is_not_remembered():
    node = explorer_stub()
    node.latest_scan = scan([2.0] * 360)
    node.scan_yaw = math.pi
    node.remember_scan_spot()
    assert node.spot_memory.spots == ()
