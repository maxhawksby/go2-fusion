"""Why did a mapping run stop? The scorecard must say.

Plain pytest, no ROS: only go2_scoreboard.accumulator and .artifacts are used.
`end_run` makes the same two calls ScoreboardRecorder._on_event makes when the
explorer shuts down (recorder.py imports rclpy, so it is not imported here).
The bag replay case makes the calls replay.py makes.
"""

import csv

from go2_scoreboard.accumulator import RunAccumulator
from go2_scoreboard.artifacts import write_artifacts

NO_SAFE_DIRECTION = (
    'no safe direction: rear and both sides lack clearance '
    'for the footprint plus margin'
)
STALE = 'stale or missing scan, costmap, or transform'


def end_run(run, event_type, detail, stamp):
    run.handle_event(event_type, 0, stamp, detail)
    run.finalize(detail or event_type)


def started_run():
    run = RunAccumulator('run')
    run.handle_event('exploration_started', 0, 0.0, 'frontier explorer started')
    run.handle_event('goal_sent', 1, 1.0, 'exploration goal')
    return run


def test_runner_stop_after_no_safe_direction_names_the_latch():
    run = started_run()
    run.handle_event('stop_latched', 1, 5.0, NO_SAFE_DIRECTION)
    end_run(run, 'exploration_stopped', 'explorer stopped', 185.0)

    assert 'no safe direction' in run.summary()['end_reason']


def test_runner_stop_after_stale_sensing_names_the_latch():
    run = started_run()
    run.handle_event('sensor_stale', 1, 5.0, STALE)
    run.handle_event('stop_latched', 1, 5.0, STALE)
    end_run(run, 'exploration_stopped', 'explorer stopped', 185.0)

    assert 'stale' in run.summary()['end_reason']


def test_scorecard_files_show_the_latch(tmp_path):
    run = started_run()
    run.handle_event('stop_latched', 1, 5.0, NO_SAFE_DIRECTION)
    end_run(run, 'exploration_stopped', 'explorer stopped', 185.0)
    write_artifacts(str(tmp_path / 'run'), run, append_index=False)

    with open(tmp_path / 'run' / 'scorecard.csv', newline='') as stream:
        row = next(csv.DictReader(stream))
    report = (tmp_path / 'run' / 'report.md').read_text()
    assert 'no safe direction' in row['end_reason']
    assert 'no safe direction' in report


def test_bag_replay_names_the_latch_once():
    run = started_run()
    run.handle_event('stop_latched', 1, 5.0, NO_SAFE_DIRECTION)
    run.handle_event('exploration_stopped', 0, 185.0, 'explorer stopped')
    run.finalize(run.end_reason or 'bag replay complete')

    assert run.summary()['end_reason'].count('no safe direction') == 1


def test_cleared_latch_is_not_blamed_for_the_stop():
    run = started_run()
    run.handle_event('stop_latched', 1, 5.0, NO_SAFE_DIRECTION)
    run.handle_event('stop_cleared', 1, 9.0, 'forward is clear')
    end_run(run, 'exploration_stopped', 'explorer stopped', 185.0)

    assert run.summary()['end_reason'] == 'explorer stopped'


def test_finished_run_keeps_its_completion_reason():
    run = started_run()
    end_run(run, 'exploration_completed',
            'No frontiers found. Exploration complete.', 120.0)

    assert run.summary()['end_reason'] == 'No frontiers found. Exploration complete.'


def test_latch_events_do_not_change_safety_stop_count():
    run = started_run()
    run.handle_event('sensor_stale', 1, 5.0, STALE)
    run.handle_event('stop_latched', 1, 5.0, STALE)
    run.handle_event('stop_latched', 1, 30.0, NO_SAFE_DIRECTION)

    assert run.summary()['safety_stops'] == 1
