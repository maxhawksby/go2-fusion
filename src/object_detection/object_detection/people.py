"""Tracks people in the odom frame, guesses where they are walking and finds
groups standing together.

The ideas come from SocialNav (https://github.com/ram95d/SocialNav, MIT): its
forecasting node keeps 8 positions 0.4 s apart and predicts 12 steps ahead, and
its group node clusters people closer than 1.5 m (DBSCAN, at least 2 people).
SocialNav's forecaster is a learned model that needs a GPU and PyTorch
Lightning; this file uses a constant-velocity forecast instead, the usual
baseline those models are compared against. No ROS here, so it tests on any
machine.
"""

import math
from collections import deque, namedtuple

# SocialNav's defaults (rl_detect_launch.py).
OBS_LEN = 8
OBS_INTERVAL_SEC = 0.4
PRED_LEN = 12
GROUP_EPS_M = 1.5
GROUP_MIN_PEOPLE = 2

# A detection this close to a track's last position continues that track.
# People walk about 1.4 m/s and the detector runs at about 5 Hz.
MATCH_RADIUS_M = 1.0
# A track not seen for this long is dropped.
TRACK_MAX_AGE_SEC = 2.0
# A person YOLO missed for up to this long is still reported, so one missed
# picture does not make them vanish.
SHOW_AGE_SEC = 1.0
# Below this speed a person counts as standing still: the position guess
# jitters by a few tenths of a meter between pictures.
MIN_WALK_SPEED_MPS = 0.2
# Velocity needs positions spread over at least this much time.
MIN_VELOCITY_SPAN_SEC = 0.5

Person = namedtuple('Person', 'id x y vx vy path group')


class Track:

    def __init__(self, track_id, t, x, y):
        self.id = track_id
        # Room for the observation window at the detector's full rate.
        self.history = deque(maxlen=64)
        self.history.append((t, x, y))

    @property
    def last(self):
        return self.history[-1]


class PeopleTracker:
    """Gives each person a steady ID while they move, from rough positions."""

    def __init__(self, match_radius=MATCH_RADIUS_M, max_age=TRACK_MAX_AGE_SEC):
        self.match_radius = match_radius
        self.max_age = max_age
        self.tracks = {}
        self.next_id = 1

    def update(self, points, now):
        """Match this picture's people ((x, y) in odom) to tracks, closest
        pairs first, and start a track for each person left over."""
        pairs = sorted(
            (math.hypot(x - track.last[1], y - track.last[2]), track_id, i)
            for track_id, track in self.tracks.items()
            for i, (x, y) in enumerate(points)
        )
        used_tracks, used_points = set(), set()
        for dist, track_id, i in pairs:
            if dist > self.match_radius:
                break
            if track_id in used_tracks or i in used_points:
                continue
            used_tracks.add(track_id)
            used_points.add(i)
            self.tracks[track_id].history.append((now, *points[i]))
        for i, (x, y) in enumerate(points):
            if i not in used_points:
                self.tracks[self.next_id] = Track(self.next_id, now, x, y)
                self.next_id += 1
        for track_id in [k for k, t in self.tracks.items() if now - t.last[0] > self.max_age]:
            del self.tracks[track_id]

    def people(self, now, obs_len=OBS_LEN, obs_interval=OBS_INTERVAL_SEC,
               pred_len=PRED_LEN, eps=GROUP_EPS_M, min_people=GROUP_MIN_PEOPLE):
        """Tracks seen in the last SHOW_AGE_SEC, with velocity, forecast
        and group number (None when alone)."""
        current = [t for t in self.tracks.values() if now - t.last[0] <= SHOW_AGE_SEC]
        labels = find_groups([t.last[1:] for t in current], eps, min_people)
        people = []
        for track, group in zip(current, labels):
            _, x, y = track.last
            vx, vy = velocity(track.history, obs_len * obs_interval)
            path = forecast(x, y, vx, vy, pred_len, obs_interval)
            people.append(Person(track.id, x, y, vx, vy, path, group))
        return people


def velocity(history, window_sec):
    """Least-squares velocity over the last window_sec of positions; zero
    when there is too little history or the person is standing."""
    t_end = history[-1][0]
    recent = [h for h in history if t_end - h[0] <= window_sec]
    if len(recent) < 2 or t_end - recent[0][0] < MIN_VELOCITY_SPAN_SEC:
        return 0.0, 0.0
    n = len(recent)
    t_mean = sum(h[0] for h in recent) / n
    x_mean = sum(h[1] for h in recent) / n
    y_mean = sum(h[2] for h in recent) / n
    var = sum((h[0] - t_mean) ** 2 for h in recent)
    vx = sum((h[0] - t_mean) * (h[1] - x_mean) for h in recent) / var
    vy = sum((h[0] - t_mean) * (h[2] - y_mean) for h in recent) / var
    if math.hypot(vx, vy) < MIN_WALK_SPEED_MPS:
        return 0.0, 0.0
    return vx, vy


def forecast(x, y, vx, vy, steps, step_sec):
    """Future positions, one every step_sec, walking straight at (vx, vy)."""
    return [(x + vx * step_sec * k, y + vy * step_sec * k) for k in range(1, steps + 1)]


def find_groups(points, eps, min_people):
    """DBSCAN on a handful of points: a group number per point, or None for a
    person standing alone."""
    neighbors = [
        [j for j, q in enumerate(points) if math.hypot(p[0] - q[0], p[1] - q[1]) <= eps]
        for p in points
    ]
    core = [len(n) >= min_people for n in neighbors]
    labels = [None] * len(points)
    group = 0
    for i in range(len(points)):
        if labels[i] is not None or not core[i]:
            continue
        labels[i] = group
        frontier = [i]
        while frontier:
            j = frontier.pop()
            if not core[j]:
                continue
            for k in neighbors[j]:
                if labels[k] is None:
                    labels[k] = group
                    frontier.append(k)
        group += 1
    return labels


def to_odom(x, y, robot_x, robot_y, robot_yaw):
    """A base_link point in the odom frame, given the robot's odom pose."""
    c, s = math.cos(robot_yaw), math.sin(robot_yaw)
    return robot_x + c * x - s * y, robot_y + s * x + c * y


def people_dict(people):
    """The JSON published on /people, rounded to centimeters."""
    return {
        f'person_{p.id}': {
            'x': round(p.x, 2), 'y': round(p.y, 2),
            'vx': round(p.vx, 2), 'vy': round(p.vy, 2),
            'group': p.group,
            'path': [[round(px, 2), round(py, 2)] for px, py in p.path],
        }
        for p in people
    }
