"""Test evidence summaries without ROS or a running simulator."""

import math
from types import SimpleNamespace as NS

from go2_sim.navigation_diagnostic import summarize


def test_summary_flags_conflicting_tf_and_keeps_unknown_rays_separate():
    transform = NS(header=NS(frame_id='odom'), child_frame_id='base_link')
    scan = NS(header=NS(frame_id='base_link'), angle_min=-math.pi,
              angle_increment=math.pi / 2, range_min=0.1, range_max=12.0,
              ranges=[2.0, float('inf'), 0.6, float('nan')])
    result = summarize([
        ('/scan', scan, 0), ('/scan', scan, 500_000_000),
        ('/tf', NS(transforms=[transform]), 0),
        ('/tf_static', NS(transforms=[transform]), 0),
        ('/cmd_vel_out', NS(linear=NS(x=0.3, y=0), angular=NS(z=0)), 0),
    ])
    assert result['topics']['/scan']['average_hz'] == 2.0
    assert result['scan_sectors']['front']['nearest_return_m'] == 0.6
    assert result['scan_sectors']['right']['finite_in_range'] == 0
    assert result['scan_sectors']['rear']['nearest_return_m'] == 2.0
    assert result['tf_edges_on_both_topics'] == [('odom', 'base_link')]
    assert result['velocity_output']['nonzero_messages'] == 1
    assert result['last_local_costmap'] is None


def test_empty_evidence_does_not_invent_measurements():
    result = summarize([])
    assert result['topics'] == {}
    assert result['scan_sectors']['front']['nearest_return_m'] is None
    assert result['last_local_costmap'] is None
