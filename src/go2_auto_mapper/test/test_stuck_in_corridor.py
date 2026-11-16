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
