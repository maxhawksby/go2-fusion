"""ROS-dependent tests of the explorer's SocialNav pipeline; run in the Humble container."""

import math
from types import MethodType, SimpleNamespace

from go2_auto_mapper.explorer import FrontierExplorer
from go2_auto_mapper.socialnav import History
from go2_auto_mapper.socialnav_follow import ANSWER_TIMEOUT_SEC, Answer

# The explorer methods these tests run for real; SocialNav, Nav2 and sensors are faked.
REAL_METHODS = (
    'start_socialnav_goal', 'tick_socialnav', 'use_socialnav_answer', 'wait_for_socialnav',
    'socialnav_step_ok', 'start_socialnav_step',
    'socialnav_blocked', 'socialnav_reached', 'clear_socialnav', 'record_goal_failure',
    'request_escape', 'tick_recovery',
)


class FakeSocialNav:
    """Answers immediately when given an Answer, never when given None."""

    def __init__(self):
        self.asked = []
        self.answer = None

    def busy(self):
        return False

    def submit(self, samples, goal_x, goal_y):
        self.asked.append((len(samples), goal_x, goal_y))
        return True

    def take_result(self):
        answer, self.answer = self.answer, None
        return answer


def answer(waypoints, pose=(0.0, 0.0, 0.0)):
    return Answer(waypoints, '', pose, 0.0)


def explorer_stub(now=100.0):
    messages, events, stops, blacklist, published = [], [], [], [], []
    clock = {'now': now}
    scan = SimpleNamespace(ranges=[5.0] * 360, angle_min=-math.pi,
                           angle_increment=2 * math.pi / 360, range_min=0.1, range_max=10.0,
                           header=SimpleNamespace(frame_id='base_link'))
    node = SimpleNamespace(
        socialnav_mode='active', socialnav_client=FakeSocialNav(), socialnav_speed=0.10,
        socialnav_history=History(), socialnav_goal=None, socialnav_points=(),
        socialnav_asked_at=None, socialnav_waiting_since=None, socialnav_holding=False,
        socialnav_idle_answers=0, socialnav_check=None,
        robot_frame='base_link', map_frame='map', latest_scan=scan, scan_age=0.1,
        socialnav_discard_next=False,
        recovery_move=None, recovery_kind='recovery', recovery_fault=None,
        backup_pending=False, scan_pending=False, scan_after_goal=True,
        goal_frontier=None, frontier_to_verify=None, verify_not_before=None,
        current_goal=None, goal_is_escape=False, consecutive_failures=0,
        stuck_failure_count=3, escape_pending=False, last_escape_time=None,
        escape_spot=None, use_spot_memory=False,
        pose=(0.0, 0.0, 0.0), path_ok=True,
        messages=messages, events=events, stops=stops, blacklist=blacklist,
        published=published, clock=clock,
    )
    logger = SimpleNamespace(info=lambda m, **k: messages.append(m),
                             warn=lambda m, **k: messages.append(m),
                             error=lambda m, **k: messages.append(m))
    node.get_logger = lambda: logger
    node.monotonic = lambda: clock['now']
    node.get_robot_pose = lambda frame=None: node.pose
    node.get_robot_position = lambda: node.pose[:2]
    node._scan_age = lambda: node.scan_age
    # A local costmap view in the same frame as the map pose: (grid, x, y, yaw, map_to_local).
    node.local_view = lambda: (SimpleNamespace(header=SimpleNamespace(frame_id='odom')),
                               *node.pose, 0.0)
    node.body_clear = lambda view, move: True
    node.request_path_check = lambda x, y: setattr(
        node, 'socialnav_check', dict(node.socialnav_check, result=node.path_ok))
    node._hold_stop = lambda: stops.append('hold')
    node._stop_for = lambda reason, refresh=None: stops.append(f'latch: {reason}')
    node.emit_event = lambda *args: events.append(args)
    node.emit_goal_outcome = lambda *args: events.append(args)
    node.log_event = lambda m: None
    node.log_goal_ended = lambda r: None
    node.blacklist_current_goal = lambda: blacklist.append(node.current_goal)
    node.goal_pub = SimpleNamespace(publish=lambda msg: published.append(msg))
    node.make_goal_pose = lambda x, y: (x, y)
    node.recovery_pub = SimpleNamespace(publish=lambda msg: published.append(msg))
    node._recovery_progress = MethodType(FrontierExplorer._recovery_progress, node)
    node.block_reason = None
    node._recovery_block_reason = lambda *args: node.block_reason
    node._finish_recovery = lambda moved: events.append(('finished', moved))
    for name in REAL_METHODS:
        setattr(node, name, MethodType(getattr(FrontierExplorer, name), node))
    for i in range(5):                       # five camera frames, one second apart
        node.socialnav_history.offer(float(i), 0.0, 0.0, 0.0, b'jpeg')
    return node


def test_a_goal_is_handed_to_socialnav_not_nav2():
    node = explorer_stub()
    assert node.start_socialnav_goal(3.0, 0.0)
    assert node.socialnav_goal == (3.0, 0.0) and node.current_goal == (3.0, 0.0)
    assert node.published == [(3.0, 0.0)]       # watchers still see the goal
    node.tick_socialnav()
    assert node.socialnav_client.asked == [(5, 3.0, 0.0)]


def test_waypoints_that_pass_every_check_start_a_strafing_slide():
    node = explorer_stub()
    node.start_socialnav_goal(3.0, 0.0)
    node.socialnav_client.answer = answer([[-0.4, 0.3]] * 5)   # 0.4 m left, 0.3 m forward
    node.tick_socialnav()                                         # answer -> path check
    node.tick_socialnav()                                         # check passed -> slide
    x, y, yaw, move, frame, started, leaving = node.recovery_move
    assert node.recovery_kind == 'socialnav'
    assert math.isclose(move[0], 0.3) and math.isclose(move[1], 0.4)
    assert frame == 'odom' and not leaving


def test_no_answer_for_30_s_holds_still_and_keeps_asking_then_resumes():
    node = explorer_stub()
    node.start_socialnav_goal(3.0, 0.0)
    node.tick_socialnav()
    node.clock['now'] += ANSWER_TIMEOUT_SEC + 1
    node.tick_socialnav()
    assert node.socialnav_holding and 'hold' in node.stops
    assert any('holding still' in m for m in node.messages)
    assert node.socialnav_goal == (3.0, 0.0)                     # goal kept
    node.socialnav_client.answer = answer([[0.0, 0.3]] * 5)
    node.tick_socialnav()
    assert not node.socialnav_holding
    assert any('answering again' in m for m in node.messages)


def test_a_failed_answer_is_asked_again():
    node = explorer_stub()
    node.start_socialnav_goal(3.0, 0.0)
    node.tick_socialnav()
    node.socialnav_client.answer = Answer(None, 'request failed: refused', (0, 0, 0), 0.0)
    node.tick_socialnav()
    node.tick_socialnav()
    assert len(node.socialnav_client.asked) == 2


def blocked_by(setup):
    node = explorer_stub()
    node.start_socialnav_goal(3.0, 0.0)
    node.socialnav_client.answer = answer([[0.0, 0.4]] * 5)
    setup(node)
    node.tick_socialnav()
    node.tick_socialnav()
    return node


def assert_goal_dropped(node):
    assert node.recovery_move is None
    assert node.blacklist == [(3.0, 0.0)]
    assert node.backup_pending                  # the unstuck move runs next
    assert node.socialnav_goal is None
    assert node.consecutive_failures == 1


def test_lidar_without_room_drops_the_goal_and_backs_up():
    node = blocked_by(lambda n: setattr(n.latest_scan, 'ranges', [0.5] * 360))
    assert_goal_dropped(node)


def test_a_blocked_body_outline_on_the_costmap_drops_the_goal():
    node = blocked_by(lambda n: setattr(n, 'body_clear', lambda view, move: False))
    assert_goal_dropped(node)


def test_no_planner_path_drops_the_goal():
    node = blocked_by(lambda n: setattr(n, 'path_ok', False))
    assert_goal_dropped(node)


def test_a_stale_scan_drops_the_goal_rather_than_moving_blind():
    node = blocked_by(lambda n: setattr(n, 'scan_age', 5.0))
    assert_goal_dropped(node)


def test_reaching_the_goal_ends_it_like_nav2_would():
    node = explorer_stub()
    node.start_socialnav_goal(0.3, 0.0)        # within 0.5 m already
    node.goal_frontier = ({(1, 1)}, 1)
    node.consecutive_failures = 2
    node.tick_socialnav()
    assert node.socialnav_goal is None
    assert node.consecutive_failures == 0 and node.scan_pending
    assert node.frontier_to_verify == ({(1, 1)}, 1)
    assert ('goal_reached', 'SocialNav steps reached goal') in node.events


def test_answers_that_never_lead_anywhere_count_as_blocked():
    node = explorer_stub()
    node.start_socialnav_goal(3.0, 0.0)
    for _ in range(3):
        node.socialnav_client.answer = answer([[0.0, 0.02]] * 5)   # all already reached
        node.tick_socialnav()
        node.tick_socialnav()
    assert_goal_dropped(node)


def test_an_obstacle_during_a_slide_drops_the_goal_instead_of_latching():
    node = explorer_stub()
    node.start_socialnav_goal(3.0, 0.0)
    node.socialnav_client.answer = answer([[0.0, 0.4]] * 5)
    node.tick_socialnav()
    node.tick_socialnav()
    node.block_reason = 'clearance lost: obstacle or unknown coverage in the move'
    node.tick_recovery()
    assert not any(s.startswith('latch') for s in node.stops)
    assert_goal_dropped(node)


def test_a_slide_drives_at_the_socialnav_speed():
    node = explorer_stub()
    node.start_socialnav_goal(3.0, 0.0)
    node.socialnav_client.answer = answer([[0.0, 0.4]] * 5)
    node.tick_socialnav()
    node.tick_socialnav()
    node.socialnav_speed = 0.07
    node.tick_recovery()
    command = node.published[-1]
    assert math.isclose(math.hypot(command.linear.x, command.linear.y), 0.07)
