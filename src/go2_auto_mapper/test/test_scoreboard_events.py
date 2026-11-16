from types import SimpleNamespace

from builtin_interfaces.msg import Time

from go2_auto_mapper.explorer import FrontierExplorer


class FailingPublisher:
    def publish(self, _message):
        raise RuntimeError('scoreboard unavailable')


def test_event_failure_does_not_escape_into_exploration_behavior():
    warnings = []
    stamp = SimpleNamespace(to_msg=lambda: Time())
    node = SimpleNamespace(
        active_goal_sequence=7,
        event_pub=FailingPublisher(),
        get_clock=lambda: SimpleNamespace(now=lambda: stamp),
        get_logger=lambda: SimpleNamespace(
            warn=lambda message, **_kwargs: warnings.append(message)),
    )

    FrontierExplorer.emit_event(node, 'goal_sent', 'measurement only', 1.0, 2.0)

    assert warnings == [
        'Could not publish automapper event goal_sent: scoreboard unavailable']


def latch_node():
    events = []
    node = SimpleNamespace(
        recovery_fault=None,
        orphans=[],
        maneuver_stopping=False,
        _ros_stamp=lambda _name: None,
        _hold_stop=lambda: None,
        emit_event=lambda event_type, detail='': events.append((event_type, detail)),
        get_logger=lambda: SimpleNamespace(error=lambda _message: None),
    )
    return node, events


def test_latched_stop_is_reported_with_its_reason():
    node, events = latch_node()

    FrontierExplorer._stop_for(node, 'no safe direction: rear lacks clearance')

    assert events == [('stop_latched', 'no safe direction: rear lacks clearance')]


def test_latched_stale_stop_keeps_the_sensor_stale_event():
    node, events = latch_node()

    FrontierExplorer._stop_for(node, 'stale or missing scan')

    assert [event_type for event_type, _ in events] == ['sensor_stale', 'stop_latched']


def test_released_stop_is_reported():
    node, events = latch_node()
    node.recovery_fault = 'no safe direction'
    node._consider_clearing_fault = lambda: setattr(node, 'recovery_fault', None)

    FrontierExplorer.watch_ahead(node)

    assert events == [('stop_cleared', 'no safe direction')]


def test_held_stop_is_not_reported_as_released():
    node, events = latch_node()
    node.recovery_fault = 'no safe direction'
    node._consider_clearing_fault = lambda: None

    FrontierExplorer.watch_ahead(node)

    assert events == []
