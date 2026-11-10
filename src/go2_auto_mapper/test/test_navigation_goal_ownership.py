"""Delayed action replies cannot finish or blacklist a different goal."""

from concurrent.futures import Future
from types import MethodType, SimpleNamespace as NS

from builtin_interfaces.msg import Time
import pytest

from go2_auto_mapper.explorer import FrontierExplorer


def make_node():
    messages, failures, outcomes = [], [], []
    node = NS(
        active_goal_sequence=2, goal_sequence=2, goal_active=True,
        shutting_down=False, current_goal_handle=object(), current_goal=(8., 9.),
        goal_sent_time=100., goal_cancelled_for_scan=False, goal_is_escape=False,
        goal_frontier=None, scan_after_goal=False, scan_pending=False,
        orphans=[], recovery_fault=None, nav_client=object(),
        _stop_for=lambda reason: messages.append(reason),
        _hold_stop=lambda: messages.append('holding stop'),
        get_logger=lambda: NS(info=messages.append, warn=messages.append, error=messages.append),
        record_goal_failure=lambda *args: failures.append(args),
        emit_goal_outcome=lambda *args: outcomes.append(args),
        log_goal_ended=lambda *args: None,
        emit_event=lambda *args: None,
    )
    for name in ('_is_current_navigation_goal', 'goal_response_callback',
                 'navigation_result_callback', '_adopt_orphan', '_orphan_result'):
        setattr(node, name, MethodType(getattr(FrontierExplorer, name), node))
    return node, failures, outcomes


def completed(value):
    future = Future()
    future.set_result(value)
    return future


@pytest.mark.parametrize('status', [4, 5, 6])
def test_old_result_never_clears_or_fails_the_new_goal(status):
    node, failures, outcomes = make_node()
    current = node.current_goal_handle
    node.navigation_result_callback(completed(NS(status=status)), sequence=1)
    assert node.current_goal_handle is current
    assert node.goal_active and node.goal_sent_time == 100.
    assert node.current_goal == (8., 9.)
    assert failures == outcomes == []


def test_timed_out_result_does_not_count_failure_twice():
    node, failures, outcomes = make_node()
    node.goal_active = False
    node.navigation_result_callback(completed(NS(status=5)), sequence=2)
    assert not failures and not outcomes


@pytest.mark.parametrize('shutdown', [False, True])
def test_late_acceptance_cancels_only_its_own_handle(shutdown):
    node, failures, outcomes = make_node()
    node.shutting_down = shutdown
    current = node.current_goal_handle
    cancellations = []
    result = Future()
    handle = NS(accepted=True, cancel_goal_async=lambda: cancellations.append(True),
                get_result_async=lambda: result)
    node.goal_response_callback(completed(handle), sequence=1)
    assert cancellations == [True]
    assert node.current_goal_handle is current
    assert not failures and not outcomes
    assert len(node.orphans) == 1
    result.set_result(NS(status=5))
    assert not node.orphans


def test_late_response_exception_does_not_fail_current_goal():
    node, failures, _ = make_node()
    future = Future()
    future.set_exception(RuntimeError('old connection ended'))
    node.goal_response_callback(future, sequence=1)
    assert node.goal_active and not failures


def test_current_goal_result_still_updates_outcome():
    node, failures, outcomes = make_node()
    node.navigation_result_callback(completed(NS(status=4)), sequence=2)
    assert outcomes == [('goal_reached', 'Nav2 reached goal')]
    assert not node.goal_active and node.current_goal_handle is None
    assert not failures


def test_response_binds_result_callback_to_its_original_sequence():
    node, failures, outcomes = make_node()
    result = Future()
    old_handle = NS(accepted=True, get_result_async=lambda: result)
    node.goal_response_callback(completed(old_handle), sequence=2)
    new_handle = object()
    node.active_goal_sequence = 3
    node.current_goal_handle = new_handle
    result.set_result(NS(status=5))
    assert node.current_goal_handle is new_handle
    assert node.goal_active and not failures and not outcomes


def test_submission_binds_response_to_sequence_at_send_time():
    node, failures, _ = make_node()
    response = Future()
    node.map_frame = 'map'
    node.get_clock = lambda: NS(now=lambda: NS(to_msg=lambda: Time()))
    node.goal_pub = NS(publish=lambda _msg: None)
    node._track_submission = lambda *_args: None
    node.nav_client = NS(server_is_ready=lambda: True, send_goal_async=lambda _goal: response)
    assert FrontierExplorer.send_goal(node, 2., 3.)
    assert node.active_goal_sequence == 3
    new_handle = object()
    node.active_goal_sequence = 4
    node.current_goal_handle = new_handle
    cancellations = []
    pending_result = Future()
    response.set_result(NS(accepted=True, cancel_goal_async=lambda: cancellations.append(True),
                           get_result_async=lambda: pending_result))
    assert cancellations == [True]
    assert node.current_goal_handle is new_handle
    assert not failures
