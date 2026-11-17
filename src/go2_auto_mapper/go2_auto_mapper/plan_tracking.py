"""How far Oak is from a Nav2 plan, for the diagnostic trace (issue #142).

Pure functions: no ROS, no state. A plan is a list of (x, y, yaw) poses in
one frame; Oak's pose is given in that same frame.
"""

import math


def plan_offset(x, y, yaw, plan):
    """(distance, heading error) from a pose to the nearest place on a plan.

    Distance is to the nearest spot on the plan's line, in meters. Heading
    error is Oak's yaw minus the plan's heading at the nearer end of that
    stretch, in (-pi, pi]; positive means Oak is turned left of the plan.
    None when the plan is empty.
    """
    best = None
    for i, (ax, ay, a_yaw) in enumerate(plan):
        bx, by, b_yaw = plan[min(i + 1, len(plan) - 1)]
        dx, dy = bx - ax, by - ay
        length_sq = dx * dx + dy * dy
        along = 0.0
        if length_sq > 0.0:
            along = min(1.0, max(0.0, ((x - ax) * dx + (y - ay) * dy) / length_sq))
        distance = math.hypot(x - (ax + along * dx), y - (ay + along * dy))
        if best is None or distance < best[0]:
            best = (distance, a_yaw if along < 0.5 else b_yaw)
    if best is None:
        return None
    error = yaw - best[1]
    return best[0], math.atan2(math.sin(error), math.cos(error))


def pick_plans(plans, now, since, min_age):
    """(latest, reference) out of `plans`, oldest first, each (time, ...).

    Only plans received at or after `since` (when the goal was sent) count.
    The reference is the newest plan at least `min_age` seconds old, or the
    goal's first plan when none is that old yet. Nav2 replans from Oak's own
    pose every second, so the latest plan always starts at Oak; the reference
    is the plan Oak was given to follow a while ago. Either is None when the
    goal has no plan.
    """
    current = [p for p in plans if p[0] >= since]
    if not current:
        return None, None
    aged = [p for p in current if now - p[0] >= min_age]
    return current[-1], aged[-1] if aged else current[0]


def format_trace(fields):
    """'Trace: name=value, ...' with floats to 2 places and None as none."""
    def text(value):
        if value is None:
            return 'none'
        return f'{value:.2f}' if isinstance(value, float) else str(value)
    return 'Trace: ' + ', '.join(f'{k}={text(v)}' for k, v in fields.items())
