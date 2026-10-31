"""Where people are and will walk, as circles the explorer keeps out of.

Reads the /people JSON from object_detection (ideas from SocialNav,
https://github.com/ram95d/SocialNav): each person's position, a 12-step
walking forecast and a group number. Three kinds of circle come out of it:

- personal space around each person,
- a narrower band along where they are about to walk,
- the space inside a group (people talking), so the robot goes around it,
  not between them.

No ROS here, so it tests on any machine.
"""

import json
import math
from collections import namedtuple

# Roughly "personal distance" (about 0.5 to 1.2 m) from proxemics.
PERSONAL_SPACE_M = 1.0
# Half-width of the band kept clear along a person's forecast walk.
PATH_SPACE_M = 0.6
# Only this much of the forecast counts; farther ahead the straight-line
# guess is mostly wrong.
PATH_HORIZON_STEPS = 6
# Margin added around a group's members.
GROUP_MARGIN_M = 0.5
# How finely a straight line is checked against the circles.
LINE_STEP_M = 0.1

Zone = namedtuple('Zone', 'x y radius kind')


def parse_people(text):
    """The /people JSON as a list of (x, y, path, group); bad entries are
    skipped instead of raising."""
    try:
        data = json.loads(text)
    except (TypeError, ValueError):
        return []
    if not isinstance(data, dict):
        return []
    people = []
    for entry in data.values():
        try:
            x, y = float(entry['x']), float(entry['y'])
            path = [(float(px), float(py)) for px, py in entry.get('path', [])]
            group = entry.get('group')
        except (KeyError, TypeError, ValueError):
            continue
        people.append((x, y, path, group))
    return people


def transform(x, y, tx, ty, yaw):
    """Point (x, y) moved by a 2D transform (tx, ty, yaw)."""
    c, s = math.cos(yaw), math.sin(yaw)
    return tx + c * x - s * y, ty + s * x + c * y


def social_zones(people, tx=0.0, ty=0.0, yaw=0.0, personal=PERSONAL_SPACE_M,
                 path_space=PATH_SPACE_M, horizon=PATH_HORIZON_STEPS,
                 margin=GROUP_MARGIN_M):
    """Circles to keep out of, moved by (tx, ty, yaw) into the map frame."""
    zones, groups = [], {}
    for x, y, path, group in people:
        mx, my = transform(x, y, tx, ty, yaw)
        zones.append(Zone(mx, my, personal, 'person'))
        for px, py in path[:horizon]:
            zones.append(Zone(*transform(px, py, tx, ty, yaw), path_space, 'path'))
        if group is not None:
            groups.setdefault(group, []).append((mx, my))
    for members in groups.values():
        cx = sum(x for x, _ in members) / len(members)
        cy = sum(y for _, y in members) / len(members)
        radius = max(math.hypot(x - cx, y - cy) for x, y in members) + margin
        zones.append(Zone(cx, cy, radius, 'group'))
    return zones


def zone_at(x, y, zones):
    """The first zone containing (x, y), or None."""
    return next((z for z in zones if math.hypot(x - z.x, y - z.y) < z.radius), None)


def line_crosses(x0, y0, x1, y1, zones, step=LINE_STEP_M):
    """The first zone the straight line from (x0, y0) to (x1, y1) enters,
    ignoring zones the start is already inside (the robot can always leave),
    or None."""
    outside = [z for z in zones if math.hypot(x0 - z.x, y0 - z.y) >= z.radius]
    length = math.hypot(x1 - x0, y1 - y0)
    steps = max(1, int(math.ceil(length / step)))
    for k in range(1, steps + 1):
        f = k / steps
        zone = zone_at(x0 + (x1 - x0) * f, y0 + (y1 - y0) * f, outside)
        if zone is not None:
            return zone
    return None
