"""
Follow SocialNav's waypoints toward the explorer's goal, one short slide at a time.

The explorer picks where to go (a frontier on the lidar map). SocialNav plans how: from the last 5
camera frames and poses it predicts 5 waypoints toward that goal ([right, forward] m from the
newest pose). The explorer checks each step (lidar room, the body outline on the costmap, a Nav2
planner path) and drives it with its own short translation on /cmd_vel_mapper, strafing straight
to the waypoint without turning. The lidar always wins: a blocked step drops the goal.

No ROS here, so it tests on any machine.
"""

from collections import namedtuple
import math
import threading
import time
import urllib.error

from go2_auto_mapper.socialnav import build_request, post, to_map

# Longest single slide toward a waypoint; the recovery checks are tuned for moves this long.
STEP_MAX_M = 0.5
# A waypoint this close counts as reached and is skipped.
WAYPOINT_REACHED_M = 0.1
# The explorer's goal counts as reached this close.
GOAL_REACHED_M = 0.5
# How long to wait for SocialNav before holding still (the learner's choice).
ANSWER_TIMEOUT_SEC = 30.0
# Room the lidar must see beyond a waypoint, as socialnav_view uses.
ROOM_MARGIN_M = 0.3
# Answers in a row with no waypoint to move to before the goal counts as blocked.
NO_PROGRESS_ANSWERS = 3

# One finished request: the 5 waypoints (None if it failed), the reason on failure, the
# (x, y, yaw) map pose they are relative to, and time.monotonic() when it finished.
Answer = namedtuple('Answer', 'waypoints reason pose finished_at')


def waypoints_to_map(waypoints, pose):
    """Return the [right, forward] waypoints as map (x, y) points from `pose` (x, y, yaw)."""
    x, y, yaw = pose
    return tuple(to_map(right, forward, x, y, yaw) for right, forward in waypoints)


def body_offset(px, py, robot_x, robot_y, robot_yaw):
    """Return map point (px, py) as (forward, left) meters from the robot."""
    dx, dy = px - robot_x, py - robot_y
    c, s = math.cos(robot_yaw), math.sin(robot_yaw)
    return c * dx + s * dy, -s * dx + c * dy


def next_step(points, pose, max_step=STEP_MAX_M, reached=WAYPOINT_REACHED_M):
    """
    Return (waypoints still ahead, the body-frame move to the first of them).

    Waypoints within `reached` of the robot are dropped. The move is a straight (forward, left)
    slide, capped at `max_step`; None when every waypoint is reached.
    """
    x, y, yaw = pose
    remaining = tuple(points)
    while remaining and math.hypot(remaining[0][0] - x, remaining[0][1] - y) < reached:
        remaining = remaining[1:]
    if not remaining:
        return (), None
    forward, left = body_offset(remaining[0][0], remaining[0][1], x, y, yaw)
    length = math.hypot(forward, left)
    if length > max_step:
        forward, left = forward * max_step / length, left * max_step / length
    return remaining, (forward, left)


def goal_reached(position, goal, tol=GOAL_REACHED_M):
    return math.hypot(goal[0] - position[0], goal[1] - position[1]) <= tol


def has_lidar_room(room):
    """
    Read socialnav.lidar_room's answer.

    None means the lidar has no return within range that way: nothing blocks it.
    """
    return room is not False


class SocialNavClient:
    """
    Ask SocialNav for waypoints on a background thread, one request at a time.

    The explorer's timer (which also drives and checks the slides) never waits on the network;
    it polls take_result() on its next loop.
    """

    def __init__(self, url, encode, timeout=ANSWER_TIMEOUT_SEC, post=post):
        self.url = url
        self.encode = encode
        self.timeout = timeout
        self.post = post
        self.lock = threading.Lock()
        self.thread = None
        self.result = None

    def busy(self):
        return self.thread is not None and self.thread.is_alive()

    def submit(self, samples, goal_x, goal_y):
        """Ask for waypoints from `samples` (5 x (t, x, y, yaw, frame), newest last)."""
        if self.busy():
            return False
        self.thread = threading.Thread(
            target=self._run, args=(list(samples), goal_x, goal_y), daemon=True
        )
        self.thread.start()
        return True

    def take_result(self):
        """Return the finished Answer once; None while waiting or if taken."""
        with self.lock:
            result, self.result = self.result, None
        return result

    def _run(self, samples, goal_x, goal_y):
        pose = samples[-1][1:4]
        try:
            reply = self.post(self.url, build_request(samples, goal_x, goal_y, self.encode),
                              self.timeout)
            waypoints = [list(map(float, w)) for w in reply['waypoints']]
            if len(waypoints) != 5 or any(len(w) != 2 for w in waypoints):
                raise ValueError(f'expected 5 [right, forward] waypoints, got {waypoints}')
            reason = ''
        except (urllib.error.URLError, OSError, ValueError, KeyError, TypeError) as e:
            waypoints, reason = None, f'request failed: {e}'
        except Exception as e:  # never let the thread die silently
            waypoints, reason = None, f'unexpected error: {e!r}'
        with self.lock:
            self.result = Answer(waypoints, reason, tuple(pose), time.monotonic())
