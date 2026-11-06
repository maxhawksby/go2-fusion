"""Stale-sensor motion stop decision. Pure Python: runs on the host too.

The module is loaded from its file so the package __init__ (which needs a
built ROS workspace) is not imported: `python3 -m pytest` on this file works
with or without ROS.
"""

import importlib.util
import pathlib

_PATH = (pathlib.Path(__file__).resolve().parents[1]
         / 'go2_robot_sdk' / 'domain' / 'sensor_watchdog.py')
_spec = importlib.util.spec_from_file_location('sensor_watchdog', _PATH)
sw = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(sw)

REQUIRED = ['odom', 'lidar']
TIMEOUT = 1.0


def fed(now, sensors=REQUIRED):
    w = sw.SensorWatchdog()
    for s in sensors:
        w.seen(s, now)
    return w


def test_never_seen_is_stale_so_oak_does_not_move_before_data():
    w = sw.SensorWatchdog()
    assert w.stale(REQUIRED, 10.0, TIMEOUT) == [('odom', None), ('lidar', None)]


def test_fresh_data_allows_motion():
    w = fed(10.0)
    assert w.stale(REQUIRED, 10.9, TIMEOUT) == []


def test_one_quiet_sensor_stops_motion():
    w = fed(10.0)
    w.seen('odom', 11.4)
    stale = w.stale(REQUIRED, 11.5, TIMEOUT)
    assert [s for s, _ in stale] == ['lidar']
    assert abs(stale[0][1] - 1.5) < 1e-9


def test_timeout_zero_turns_the_check_off():
    assert sw.SensorWatchdog().stale(REQUIRED, 10.0, 0.0) == []


def test_startup_gives_no_event_until_first_fresh():
    w = sw.SensorWatchdog()
    assert w.update(REQUIRED, 1.0, TIMEOUT) is None   # nothing yet: no event
    w.seen('odom', 2.0)
    w.seen('lidar', 2.0)
    assert w.update(REQUIRED, 2.1, TIMEOUT) is None   # first fresh: no event


def test_stale_then_fresh_events_fire_once_each():
    w = fed(10.0)
    assert w.update(REQUIRED, 10.5, TIMEOUT) is None
    w.seen('odom', 11.0)
    event = w.update(REQUIRED, 11.2, TIMEOUT)
    assert event.name == sw.STALE
    assert event.sensors == ['lidar']
    assert abs(event.age_s - 1.2) < 1e-9
    assert w.update(REQUIRED, 12.0, TIMEOUT) is None  # still stale: no repeat

    w.seen('odom', 14.0)
    w.seen('lidar', 14.0)
    event = w.update(REQUIRED, 14.1, TIMEOUT)
    assert event.name == sw.FRESH
    assert event.sensors == ['lidar']
    assert abs(event.stopped_s - 2.9) < 1e-9
    assert w.update(REQUIRED, 14.2, TIMEOUT) is None  # still fresh: no repeat


def test_stale_event_names_every_quiet_sensor_and_oldest_age():
    w = fed(10.0)
    w.update(REQUIRED, 10.1, TIMEOUT)
    w.seen('lidar', 10.5)
    event = w.update(REQUIRED, 12.0, TIMEOUT)
    assert event.sensors == ['odom', 'lidar']
    assert abs(event.age_s - 2.0) < 1e-9


def test_a_newly_required_sensor_never_seen_reports_no_age():
    w = fed(10.0)
    w.update(REQUIRED, 10.1, TIMEOUT)
    event = w.update(REQUIRED + ['scan'], 10.2, TIMEOUT)
    assert event.name == sw.STALE
    assert event.sensors == ['scan']
    assert event.age_s is None
