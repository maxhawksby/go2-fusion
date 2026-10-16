"""ROS-dependent tests; run inside the Humble container."""

import time
from types import SimpleNamespace

from go2_auto_mapper.explorer import FrontierExplorer


def explorer_stub():
    node = SimpleNamespace(observed={}, recovery_move=None, recovery_fault=False)
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
    node.refresh_after = (98.0, time.monotonic()-21)
    messages = []
    node.get_logger = lambda: SimpleNamespace(error=messages.append)
    FrontierExplorer._exploration_loop_impl(node)
    assert node.recovery_fault
    assert messages
