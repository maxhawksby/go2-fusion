import math

import pytest

from oak_corridor import Sim

REFUSED = 6
SUCCEEDED = 4


@pytest.mark.parametrize('start_yaw_deg, scan_turn_deg', [
    (0.0, 60.0),   # scan refused after Oak turned 60 degrees
    (60.0, 0.0),   # Oak already idle at 60 degrees, wall at the nose
], ids=['scan-refused-part-way', 'idle-near-wall'])
def test_refused_turns_do_not_stop_exploring_forever(start_yaw_deg, scan_turn_deg):
    sim = Sim(spin_status=REFUSED, start=(0.0, 0.0, math.radians(start_yaw_deg)),
              scan_turns_before_abort=math.radians(scan_turn_deg))
    report = sim.run(120.0)
    assert not report['collided'], report
    assert report['navigation_goals'] >= 1, report


def test_control_turns_that_succeed_lead_to_a_goal():
    sim = Sim(spin_status=SUCCEEDED, start=(0.0, 0.0, math.radians(60.0)))
    report = sim.run(120.0)
    assert not report['collided'], report
    assert report['navigation_goals'] >= 1, report


def test_a_stop_for_missing_scan_waits_for_the_dead_cloud():
    sim = Sim(spin_status=REFUSED, cloud_dies_at=1.0)
    sim.run(3.0)
    sim.node.latest_scan = None
    sim.node.start_backup()
    goals_before = len(sim.nav_goals)
    report = sim.run(60.0)
    assert report['still_stopped'], report
    assert len(sim.nav_goals) == goals_before, sim.nav_goals


def test_a_stop_holds_while_the_spin_server_is_down():
    sim = Sim(spin_ready=False)
    report = sim.run(30.0)
    assert report['still_stopped'], report
    assert report['navigation_goals'] == 0, report


def test_a_recovery_move_walks_oak_past_its_deadband():
    # Oak stands still for walking commands under about 0.15 m/s
    # (go2_sim/config/sim.yaml min_walk_speed, from the 2026-11-16 rosbags).
    sim = Sim(spin_status=SUCCEEDED, start=(0.0, 0.0, 0.0), min_walk_speed=0.15)
    sim.run(3.0)
    sim.node.start_backup()
    report = sim.run(20.0)
    assert not report['collided'], report
    assert report['pose'][0] <= -0.25, report
    assert not any('time limit' in msg for _, msg in sim.node.log), sim.node.log
