"""The diagnostic trace (issue #142) logs and does nothing else."""

import math
import re
from types import MethodType, SimpleNamespace as NS

from oak_corridor import Sim
from ros_stubs import load_explorer

ex = load_explorer()


def quaternion(yaw):
    return NS(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def path_message(points, frame='map'):
    return NS(header=NS(frame_id=frame), poses=[
        NS(pose=NS(position=NS(x=x, y=y), orientation=quaternion(yaw)))
        for x, y, yaw in points])


def make_node(pose=(1.0, 0.3, 0.5), goal_active=True):
    log = []
    node = NS(
        goal_active=goal_active, goal_sent_time=ex.time.monotonic() - 2.0,
        map_frame='map', obstacle_stop_distance=0.3,
        trace_plans=ex.collections.deque(maxlen=ex.TRACE_PLANS_KEPT), trace_cmd=None,
        get_robot_pose=lambda frame: pose,
        local_view=lambda: None,
        get_logger=lambda: NS(info=lambda m: log.append(('info', m)),
                              error=lambda m: log.append(('error', m))),
    )
    for name in ('trace', '_trace_on_plan', '_trace_on_cmd', '_trace_plan_fields'):
        setattr(node, name, MethodType(getattr(ex.FrontierExplorer, name), node))
    return node, log


def fields(line):
    return dict(re.findall(r'(\w+)=([^,\s]+)', line))


def test_trace_line_reports_pose_command_and_offset_from_the_plan():
    node, log = make_node()
    node._trace_on_plan(path_message([(0.0, 0.0, 0.0), (2.0, 0.0, 0.0)]))
    node._trace_on_cmd(NS(linear=NS(x=0.25), angular=NS(z=-0.1)))
    node.trace()
    (level, line), = log
    assert level == 'info' and line.startswith('Trace: t=')
    got = fields(line)
    assert (got['x'], got['y'], got['yaw']) == ('1.00', '0.30', '0.50')
    assert (got['cmd_v'], got['cmd_w']) == ('0.25', '-0.10')
    assert (got['plan_dist'], got['plan_yaw_err']) == ('0.30', '0.50')
    assert (got['ref_dist'], got['ref_yaw_err']) == ('0.30', '0.50')
    assert got['gap'] == 'none'


def test_trace_without_a_plan_or_command_says_none():
    node, log = make_node()
    node.trace()
    got = fields(log[0][1])
    assert got['plan_dist'] == got['plan_yaw_err'] == got['cmd_v'] == 'none'


def test_no_trace_line_without_an_active_goal():
    node, log = make_node(goal_active=False)
    node.trace()
    assert log == []


def test_a_trace_failure_is_logged_not_raised():
    node, log = make_node()

    def broken(frame):
        raise RuntimeError('tf gone')
    node.get_robot_pose = broken
    node.trace()
    assert log == [('error', 'Trace error: tf gone')]


def test_trace_is_off_by_default():
    node = Sim().node
    assert node.diagnostic_trace is False
    assert not hasattr(node, 'trace_timer')
    assert all(callback != node.trace for _, callback, _ in node._timers)
