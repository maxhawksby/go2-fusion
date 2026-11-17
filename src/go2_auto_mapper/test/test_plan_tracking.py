import math

import pytest

from go2_auto_mapper.plan_tracking import format_trace, pick_plans, plan_offset

# A plan that runs 2 m east along y = 0, then 2 m north along x = 2.
EAST = 0.0
NORTH = math.pi / 2.0
L_PLAN = [(0.0, 0.0, EAST), (1.0, 0.0, EAST), (2.0, 0.0, NORTH), (2.0, 2.0, NORTH)]


def test_on_the_plan_and_facing_along_it_is_no_offset():
    distance, yaw_error = plan_offset(0.5, 0.0, EAST, L_PLAN)
    assert distance == pytest.approx(0.0)
    assert yaw_error == pytest.approx(0.0)


def test_distance_is_measured_to_the_line_between_plan_poses():
    distance, _ = plan_offset(0.5, 0.3, EAST, L_PLAN)
    assert distance == pytest.approx(0.3)


def test_distance_past_the_end_is_to_the_last_pose():
    distance, _ = plan_offset(2.0, 3.0, NORTH, L_PLAN)
    assert distance == pytest.approx(1.0)


def test_heading_error_is_signed_left_positive():
    _, left = plan_offset(0.2, 0.0, 0.4, L_PLAN)
    _, right = plan_offset(0.2, 0.0, -0.4, L_PLAN)
    assert left == pytest.approx(0.4)
    assert right == pytest.approx(-0.4)


def test_heading_error_uses_the_plan_heading_where_oak_is():
    _, yaw_error = plan_offset(2.0, 1.5, EAST, L_PLAN)
    assert yaw_error == pytest.approx(-math.pi / 2.0)


def test_heading_error_wraps_across_pi():
    plan = [(0.0, 0.0, 3.0), (-1.0, 0.0, 3.0)]
    _, yaw_error = plan_offset(0.0, 0.0, -3.0, plan)
    assert yaw_error == pytest.approx(2.0 * math.pi - 6.0)


def test_a_reversing_stretch_uses_the_pose_heading_not_the_travel_direction():
    backing_west = [(1.0, 0.0, EAST), (0.0, 0.0, EAST)]
    _, yaw_error = plan_offset(0.5, 0.0, EAST, backing_west)
    assert yaw_error == pytest.approx(0.0)


def test_single_pose_plan():
    distance, yaw_error = plan_offset(3.0, 4.0, 0.5, [(0.0, 0.0, 0.0)])
    assert distance == pytest.approx(5.0)
    assert yaw_error == pytest.approx(0.5)


def test_empty_plan_has_no_offset():
    assert plan_offset(0.0, 0.0, 0.0, []) is None


def test_reference_is_the_newest_plan_old_enough():
    plans = [(10.0, 'a'), (11.0, 'b'), (14.0, 'c'), (15.5, 'd')]
    latest, reference = pick_plans(plans, now=16.0, since=10.0, min_age=5.0)
    assert latest[1] == 'd'
    assert reference[1] == 'b'


def test_reference_falls_back_to_the_goals_first_plan():
    plans = [(10.0, 'a'), (11.0, 'b')]
    latest, reference = pick_plans(plans, now=12.0, since=10.0, min_age=5.0)
    assert latest[1] == 'b'
    assert reference[1] == 'a'


def test_plans_from_before_the_goal_do_not_count():
    plans = [(1.0, 'old goal'), (2.0, 'old goal')]
    assert pick_plans(plans, now=20.0, since=10.0, min_age=5.0) == (None, None)
    plans.append((10.5, 'new goal'))
    latest, reference = pick_plans(plans, now=20.0, since=10.0, min_age=5.0)
    assert latest[1] == reference[1] == 'new goal'


def test_trace_line_format():
    line = format_trace({'t': 12.5, 'x': -1.0, 'gap': None, 'cmd_v': 0.25})
    assert line == 'Trace: t=12.50, x=-1.00, gap=none, cmd_v=0.25'
