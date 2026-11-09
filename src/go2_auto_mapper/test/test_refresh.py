"""ROS-dependent tests; run inside the Humble container."""

import time
from types import SimpleNamespace

from go2_auto_mapper.explorer import FrontierExplorer


def explorer_stub():
    node = SimpleNamespace(
        observed={},
        recovery_move=None,
        recovery_fault=False,
        supervised_lab_mode=False,
        lab_paused=False,
    )
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=100_000_000_000))
    node.data_fresh = lambda names, after=None: FrontierExplorer.data_fresh(
        node, names, after)
    return node


def test_refresh_requires_every_stream_after_scan():
    node = explorer_stub()
    for name in ('scan', 'cloud', 'map', 'costmap'):
        node.observed[name] = (time.monotonic(), 99.0)
    assert node.data_fresh(tuple(node.observed), 98.0)
    node.observed['map'] = (time.monotonic(), 97.0)
    assert not node.data_fresh(tuple(node.observed), 98.0)


def test_recent_delivery_of_old_data_is_stale():
    node = explorer_stub()
    node.observed['scan'] = (time.monotonic(), 90.0)
    assert not node.data_fresh(('scan',))
    node.observed['scan'] = (time.monotonic()-5, 99.0)
    assert not node.data_fresh(('scan',))


def test_refresh_timeout_latches_stop():
    node = explorer_stub()
    # (stop ROS time, monotonic start, need a new SLAM map), timed out.
    node.refresh_after = (98.0, time.monotonic() - 21, False)
    node.tf_stamps = {}
    messages = []
    node.get_logger = lambda: SimpleNamespace(
        error=messages.append, info=messages.append)
    node._note_tf_stamps = lambda: None
    node._hold_stop = lambda: None
    for name in ('_refresh_check', '_ros_stamp', '_stop_for'):
        method = getattr(FrontierExplorer, name)
        setattr(node, name, lambda *a, m=method, **k: m(node, *a, **k))
    FrontierExplorer._exploration_loop_impl(node)
    assert node.recovery_fault
    assert messages
    # The latch keeps the sensing requirement, so a clear way ahead alone
    # can't release it.
    assert node.fault_refresh == (98.0, False)
