"""obstacle_watch_cancels_goals decides whether an obstacle ahead ends a goal."""

from types import MethodType, SimpleNamespace as NS

import pytest

from ros_stubs import load_explorer

ex = load_explorer()

GAP_M = 0.1
DRIVING_FORWARD_MPS = 0.2


@pytest.fixture
def obstacle_ahead(monkeypatch):
    monkeypatch.setattr(ex, 'free_distance_ahead', lambda *_: GAP_M)
    monkeypatch.setattr(ex, 'translation_open', lambda *_: True)


def make_node(cancels_goals, goal_active=True):
    did = []
    grid = NS(header=NS(frame_id='odom'))
    handle = NS(cancel_goal_async=lambda: did.append('cancel'))
    node = NS(
        obstacle_watch_cancels_goals=cancels_goals,
        goal_active=goal_active, current_goal_handle=handle if goal_active else None,
        recovery_fault=None, orphans=[], maneuver_stopping=False, recovery_move=None,
        refresh_after=None, maneuver_active=False, yield_for_people=lambda: False,
        local_view=lambda: (grid, 0.0, 0.0, 0.0, None),
        _forward_speed=lambda *_: DRIVING_FORWARD_MPS,
        obstacle_stop_distance=0.3, backup_distance=0.3, blocked_timeout=0.0,
        blocked_since=None, blocked_heading=None, blocked_time=None,
        backup_pending=False, diagnostic_trace=False, map_frame='map',
        get_robot_pose=lambda frame: (1.0, 2.0, 0.0),
        get_logger=lambda: NS(info=lambda m: None, warn=lambda m: None,
                              error=lambda m: did.append('error: ' + m)),
        request_escape=lambda reason: did.append('escape'),
        back_away_while_idle=lambda gap: did.append('back away'),
    )
    node.watch_ahead = MethodType(ex.FrontierExplorer.watch_ahead, node)
    return node, did, handle


def watch_twice(node):
    # blocked_timeout 0.0 cancels on the second tick that sees the obstacle.
    node.watch_ahead()
    node.watch_ahead()


def test_by_default_an_obstacle_ahead_cancels_the_goal(obstacle_ahead):
    node, did, _ = make_node(cancels_goals=True)
    watch_twice(node)
    assert did == ['cancel', 'escape']
    assert node.current_goal_handle is None and node.backup_pending


def test_with_the_flag_off_the_goal_is_kept(obstacle_ahead):
    node, did, handle = make_node(cancels_goals=False)
    watch_twice(node)
    assert did == []
    assert node.current_goal_handle is handle and not node.backup_pending


def test_with_the_flag_off_an_idle_oak_still_backs_away(obstacle_ahead):
    node, did, _ = make_node(cancels_goals=False, goal_active=False)
    node.watch_ahead()
    assert did == ['back away']
