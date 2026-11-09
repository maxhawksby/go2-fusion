import argparse
import math

import pytest

from go2_sim.trials import (distance_increment, distance_update, explorer_command,
                            explorer_parameter, scoreboard_parity, variant_name)


def test_distance_increment_rejects_reset_or_teleport():
    assert distance_increment(None, (1.0, 2.0)) == 0.0
    assert math.isclose(distance_increment((1.0, 2.0), (1.3, 2.4)), 0.5)
    assert distance_increment((1.0, 2.0), (3.0, 2.0)) == 0.0


def test_distance_update_ignores_motion_before_measurement_start():
    total, previous = distance_update(0.0, (0.0, 0.0), (3.0, 0.0), False)
    assert total == 0.0
    assert previous == (3.0, 0.0)
    total, previous = distance_update(total, previous, (3.3, 0.4), True)
    assert math.isclose(total, 0.5)
    assert previous == (3.3, 0.4)


def test_parity_accepts_documented_tolerances():
    legacy = {'distance_m': 7.0, 'odom_distance_m': 10.0,
              'completeness_pct': 75.0, 'collisions': 2}
    score = {'distance_m': 10.8, 'completeness_pct': 74.2, 'collisions': 2}
    checks, failures = scoreboard_parity(legacy, score)
    assert all(checks.values())
    assert failures == []


def test_parity_names_failed_metrics():
    legacy = {'distance_m': 10.0, 'odom_distance_m': 10.0,
              'completeness_pct': 75.0, 'collisions': 2}
    score = {'distance_m': 12.0, 'completeness_pct': 72.0, 'collisions': 3}
    _checks, failures = scoreboard_parity(legacy, score)
    assert failures == ['distance', 'completeness', 'collisions']


def test_parity_handles_nullable_scoreboard_fields():
    legacy = {'distance_m': 1.0, 'odom_distance_m': 1.0,
              'completeness_pct': 10.0, 'collisions': 0}
    score = {'distance_m': 1.0, 'completeness_pct': None, 'collisions': 0}
    checks, failures = scoreboard_parity(legacy, score)
    assert not any(checks.values())
    assert failures == ['scoreboard metrics unavailable']


def test_parity_compares_reported_odom_not_hidden_ground_truth():
    legacy = {'distance_m': 20.0, 'odom_distance_m': 10.0,
              'completeness_pct': 75.0, 'collisions': 0}
    score = {'distance_m': 10.0, 'completeness_pct': 75.0, 'collisions': 0}
    checks, failures = scoreboard_parity(legacy, score)
    assert all(checks.values())
    assert failures == []


def test_explorer_command_adds_only_explicit_parameter_overrides():
    assert explorer_command([]) == [
        'ros2', 'run', 'go2_auto_mapper', 'explorer'
    ]
    assert explorer_command(['blocked_timeout:=2.0', 'scan_after_goal:=false']) == [
        'ros2', 'run', 'go2_auto_mapper', 'explorer', '--ros-args',
        '-p', 'blocked_timeout:=2.0', '-p', 'scan_after_goal:=false',
    ]


def test_experiment_labels_and_parameters_reject_cli_fragments():
    assert explorer_parameter('blocked_timeout:=2.0') == 'blocked_timeout:=2.0'
    assert variant_name('blocked-grace_2s') == 'blocked-grace_2s'
    with pytest.raises(argparse.ArgumentTypeError):
        explorer_parameter('--ros-args')
    with pytest.raises(argparse.ArgumentTypeError):
        variant_name('../other-run')
