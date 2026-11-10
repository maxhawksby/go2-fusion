from types import SimpleNamespace

from go2_auto_mapper.explorer import FrontierExplorer


class Logger:
    def __init__(self):
        self.messages = []

    def error(self, message):
        self.messages.append(message)

    def info(self, message):
        self.messages.append(message)


def test_failed_goal_pauses_in_supervised_mode():
    paused = []
    outcomes = []
    node = SimpleNamespace(
        supervised_lab_mode=True,
        goal_frontier=object(),
        goal_is_escape=False,
        blacklist_current_goal=lambda: None,
        emit_goal_outcome=lambda *args: outcomes.append(args),
        _pause_for_lab=paused.append,
    )

    FrontierExplorer.record_goal_failure(node, 'Nav2 result status 6')

    assert node.goal_frontier is None
    assert outcomes == [('goal_failed', 'Nav2 result status 6')]
    assert paused == ['Nav2 result status 6; waiting for an operator resume request']


def test_paused_lab_loop_holds_zero_velocity():
    holds = []
    node = SimpleNamespace(
        supervised_lab_mode=True,
        lab_paused=True,
        _hold_stop=lambda: holds.append(True),
    )

    FrontierExplorer._exploration_loop_impl(node)

    assert holds == [True]


def test_resume_requires_supervised_mode_and_clears_pause():
    response = SimpleNamespace(success=None, message='')
    node = SimpleNamespace(
        supervised_lab_mode=True,
        lab_paused=True,
        lab_pause_reason='test failure',
        maneuver_active=False,
        orphans=[],
        maneuver_stopping=False,
        get_logger=lambda: Logger(),
    )

    result = FrontierExplorer.handle_resume_request(node, None, response)

    assert result.success
    assert not node.lab_paused
    assert node.lab_pause_reason is None
