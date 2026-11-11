"""Offline windows retain old replies without charging them to the new goal."""

from types import SimpleNamespace as NS

from go2_sim.navigation_diagnostic import goal_timeline


def event(kind, sequence, detail=''):
    return NS(event_type=kind, goal_sequence=sequence, detail=detail)


def odom(x):
    return NS(pose=NS(pose=NS(position=NS(x=x, y=0.0))))


def test_goal_motion_timeout_and_late_reply_remain_separate():
    result = goal_timeline([
        ('/auto_mapper/events', event('goal_sent', 1), 1_000_000_000),
        ('/auto_mapper/events', event('goal_accepted', 1), 2_000_000_000),
        ('/odom', odom(0.0), 2_100_000_000),
        ('/odom', odom(0.5), 3_000_000_000),
        ('/auto_mapper/events', event('goal_failed', 1, 'timeout'), 61_000_000_000),
        ('/auto_mapper/events', event('goal_sent', 2), 62_000_000_000),
        ('/auto_mapper/events', event('goal_failed', 1, 'late result'), 63_000_000_000),
        ('/auto_mapper/events', event('goal_reached', 2), 65_000_000_000),
    ])
    first, second = result['goals']
    assert first['duration_s'] == 60 and first['distance_m'] == .5
    assert first['detail'] == 'timeout'
    assert second['outcome'] == 'goal_reached' and second['duration_s'] == 3
    assert any(item.get('detail') == 'late result' for item in result['events'])


def test_supersession_and_run_stop_are_not_invented_successes():
    result = goal_timeline([
        ('/auto_mapper/events', event('goal_sent', 1), 1_000_000_000),
        ('/auto_mapper/events', event('goal_sent', 2), 2_000_000_000),
        ('/auto_mapper/events', event('exploration_stopped', 2), 4_000_000_000),
    ])
    assert [g['outcome'] for g in result['goals']] == ['superseded_without_outcome', 'run_stopped']
