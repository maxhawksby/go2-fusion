"""Experimental final goal orientation, independent of frontier selection."""

import math


def validate_heading_mode(mode):
    if mode not in ('map_zero', 'toward_goal'):
        raise ValueError('goal_heading_mode must be map_zero or toward_goal')
    return mode


def goal_yaw(mode, pose, goal_x, goal_y):
    """Return None to defer a goal when its required map-frame pose is absent."""
    validate_heading_mode(mode)
    if mode == 'map_zero':
        return 0.0
    if pose is None or not all(math.isfinite(value) for value in pose):
        return None
    start_x, start_y, current_yaw = pose
    dx, dy = goal_x - start_x, goal_y - start_y
    return current_yaw if math.hypot(dx, dy) < 1e-6 else math.atan2(dy, dx)
