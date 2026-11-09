import math

from go2_scoreboard.accumulator import COMPLETE, FINALIZING, RunAccumulator


def test_distance_rejects_odometry_reset_and_warns_once():
    run = RunAccumulator('run', max_odom_step_m=1.0)
    run.handle_event('exploration_started', 0, 10.0)
    run.update_odom(0.0, 0.0, 10.0)
    run.update_odom(0.3, 0.4, 11.0)
    run.update_odom(20.0, 20.0, 12.0)
    run.update_odom(20.3, 20.4, 13.0)

    assert math.isclose(run.distance_m, 1.0)
    assert len(run.warnings) == 1


def test_events_are_deduplicated_and_have_one_outcome_per_goal():
    run = RunAccumulator('run')
    assert run.handle_event('goal_sent', 4, 1.0, 'target')
    assert not run.handle_event('goal_sent', 4, 1.0, 'target')
    run.handle_event('goal_reached', 4, 2.0)
    run.handle_event('goal_failed', 4, 3.0)
    run.handle_event('recovery_started', 0, 3.5)
    run.handle_event('sensor_stale', 0, 4.0)

    summary = run.summary()
    assert summary['goals_sent'] == 1
    assert summary['goals_reached'] == 1
    assert summary['goals_failed'] == 0
    assert summary['recoveries'] == 1
    assert summary['safety_stops'] == 1


def test_finalization_is_idempotent():
    run = RunAccumulator('run')
    run.start(1.0)
    assert run.begin_finalizing()
    assert run.state == FINALIZING
    assert run.finalize('done')
    assert run.state == COMPLETE
    assert not run.finalize('changed')
    assert run.end_reason == 'done'
