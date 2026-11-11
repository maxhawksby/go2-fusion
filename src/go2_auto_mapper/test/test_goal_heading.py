"""Heading policy geometry and the actual action submission boundary."""

from concurrent.futures import Future
import math
from types import SimpleNamespace as NS

from builtin_interfaces.msg import Time
import pytest

from go2_auto_mapper.explorer import FrontierExplorer
from go2_auto_mapper.goal_heading import goal_yaw, validate_heading_mode


@pytest.mark.parametrize('x,y,yaw', [(1, 0, 0), (0, 1, math.pi / 2),
                                     (-1, 0, math.pi), (0, -1, -math.pi / 2)])
def test_cardinal_directions_and_translation(x, y, yaw):
    assert goal_yaw('toward_goal', (10, 20, .7), 10 + x, 20 + y) == pytest.approx(yaw)


def test_coincident_goal_keeps_current_yaw():
    assert goal_yaw('toward_goal', (2, 3, .73), 2, 3) == .73


def test_control_retains_zero_without_pose_and_candidate_defers():
    assert goal_yaw('map_zero', None, 1, 2) == 0
    assert goal_yaw('toward_goal', None, 1, 2) is None
    assert goal_yaw('toward_goal', (math.nan, 0, 0), 1, 2) is None
    with pytest.raises(ValueError, match='goal_heading_mode'):
        validate_heading_mode('invented')


@pytest.mark.parametrize('escape', [False, True])
@pytest.mark.parametrize('mode', ['map_zero', 'toward_goal'])
def test_action_goal_orientation_and_sequence_for_both_goal_kinds(escape, mode):
    submitted, logs = [], []
    def submit(goal):
        submitted.append(goal)
        return Future()
    node = NS(
        nav_client=NS(server_is_ready=lambda: True, send_goal_async=submit),
        map_frame='map', goal_heading_mode=mode, get_robot_pose=lambda _frame: (1, 2, .7),
        get_clock=lambda: NS(now=lambda: NS(to_msg=lambda: Time())),
        get_logger=lambda: NS(info=logs.append, warn=logs.append),
        goal_sequence=4, emit_event=lambda *_args: None,
        goal_pub=NS(publish=lambda _pose: None), _track_submission=lambda *_args: None,
    )
    assert FrontierExplorer.send_goal(node, 1., 5., is_escape=escape)
    q = submitted[0].pose.pose.orientation
    expected = 0 if mode == 'map_zero' else math.pi / 2
    assert 2 * math.atan2(q.z, q.w) == pytest.approx(expected)
    assert node.active_goal_sequence == 5 and node.goal_is_escape == escape
    assert any('sequence=5' in line and 'yaw=' in line for line in logs)


def test_missing_pose_sends_nothing_and_does_not_advance_sequence():
    node = NS(nav_client=NS(server_is_ready=lambda: True), map_frame='map',
              goal_heading_mode='toward_goal', get_robot_pose=lambda _frame: None,
              goal_sequence=9, get_logger=lambda: NS(warn=lambda _text: None))
    assert not FrontierExplorer.send_goal(node, 1, 2)
    assert node.goal_sequence == 9
