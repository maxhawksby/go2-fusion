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
