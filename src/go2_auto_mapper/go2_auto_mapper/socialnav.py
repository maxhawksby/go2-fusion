"""
Helpers for asking the SocialNav model for a path.

Helpers for asking the SocialNav model (host_tools/socialnav/server.py) for a path: a short memory
of camera frames and poses, and the conversions between the map frame and the model's robot frame
of [right, forward] meters.

No ROS here, so it tests on any machine.
"""

import base64
import json
import math
import urllib.request
from collections import deque

# The model looks at this many past frames and positions (the paper's n = 5).
HISTORY = 5
# How far apart in time the remembered frames are.
STEP_SEC = 1.0
# The model works in "steps"; positions are divided by this. With the robot
# standing still there is no step to measure, so this floor keeps the goal
# from blowing up. A guess: the training value per robot is not published.
MIN_STEP_SCALE_M = 0.3


class History:
    """The last HISTORY (time, x, y, yaw, frame) samples, STEP_SEC apart."""

    def __init__(self, length=HISTORY, step_sec=STEP_SEC):
        self.samples = deque(maxlen=length)
        self.step_sec = step_sec

    def offer(self, t, x, y, yaw, frame):
        """Keep this sample if a step has passed since the last kept one."""
        if self.samples and t - self.samples[-1][0] < self.step_sec:
            return False
        self.samples.append((t, x, y, yaw, frame))
        return True

    def full(self):
        return len(self.samples) == self.samples.maxlen


def to_robot(x, y, robot_x, robot_y, robot_yaw):
    """Express a map point as [right, forward] meters from the robot."""
    dx, dy = x - robot_x, y - robot_y
    c, s = math.cos(robot_yaw), math.sin(robot_yaw)
    forward = c * dx + s * dy
    left = -s * dx + c * dy
    return [-left, forward]


def to_map(right, forward, robot_x, robot_y, robot_yaw):
    """Turn a [right, forward] point from the robot back into the map frame."""
    c, s = math.cos(robot_yaw), math.sin(robot_yaw)
    left = -right
    return robot_x + c * forward - s * left, robot_y + s * forward + c * left


def step_scale(samples):
    """Mean distance between remembered positions, but at least MIN_STEP_SCALE_M."""
    steps = [math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(samples, list(samples)[1:])]
    mean = sum(steps) / len(steps) if steps else 0.0
    return max(mean, MIN_STEP_SCALE_M)


def build_request(samples, goal_x, goal_y, encode):
    """
    Build the /waypoints body from the history (newest last) and a map-frame goal.

    encode turns a frame into JPEG bytes; the server takes them as base64.
    """
    _, x, y, yaw, _ = samples[-1]
    return {
        'images': [base64.b64encode(encode(s[4])).decode('ascii') for s in samples],
        'history': [to_robot(s[1], s[2], x, y, yaw) for s in samples],
        'goal': to_robot(goal_x, goal_y, x, y, yaw),
        'step_scale': step_scale(samples),
    }


def post(url, body, timeout):
    request = urllib.request.Request(
        url, data=json.dumps(body).encode('utf-8'), headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(request, timeout=timeout) as response:
        return json.loads(response.read().decode('utf-8'))


def lidar_room(ranges, angle_min, angle_increment, right, forward, margin):
    """
    Check whether the lidar sees room out to a waypoint.

    True if no lidar reading is closer than the waypoint (plus margin) along its direction; None if
    the scan has nothing that way.
    """
    bearing = math.atan2(-right, forward)
    distance = math.hypot(right, forward)
    nearest = None
    for i, reading in enumerate(ranges):
        if not math.isfinite(reading) or reading <= 0.0:
            continue
        off = (angle_min + i * angle_increment - bearing + math.pi) % (2.0 * math.pi) - math.pi
        if abs(off) <= math.radians(10.0) and (nearest is None or reading < nearest):
            nearest = reading
    if nearest is None:
        return None
    return nearest >= distance + margin
