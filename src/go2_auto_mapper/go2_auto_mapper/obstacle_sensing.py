"""Reads Nav2's local costmap to tell what is in front of the robot.

The local costmap (/local_costmap/costmap) is built from /scan, so this sees
the same obstacles the controller avoids.
Functions take the grid and a pose in the grid's frame (odom); they don't
touch ROS, so they can be tested with a hand-made grid.
"""

import math

# nav_msgs/OccupancyGrid form of a Nav2 costmap: 100 is a lethal cell (an
# obstacle itself), 99 inscribed, 1-98 inflation, -1 unknown.
LETHAL_COST = 100

# The robot's outline (nav2_params.yaml footprint): the nose is 0.36 m ahead
# of base_link and the body 0.4 m wide. The margin keeps the checked strip a
# little wider than the body, so walls it would scrape count too.
ROBOT_FRONT_M = 0.36
ROBOT_HALF_WIDTH_M = 0.2
SIDE_MARGIN_M = 0.05


def cost_at(grid, x, y):
    """Cost at world point (x, y) in the grid's frame, or None off the grid."""
    info = grid.info
    gx = int(math.floor((x - info.origin.position.x) / info.resolution))
    gy = int(math.floor((y - info.origin.position.y) / info.resolution))
    if not (0 <= gx < info.width and 0 <= gy < info.height):
        return None
    return grid.data[gy * info.width + gx]


def first_obstacle_along(grid, x, y, heading, start, length,
                         half_width=ROBOT_HALF_WIDTH_M + SIDE_MARGIN_M):
    """Distance past `start` to the first lethal cell in a strip `2 *
    half_width` wide, running from (x, y) along `heading` (radians), checked
    out to `start + length`. None if the strip is clear.

    Cells off the grid or unknown count as clear: this is about obstacles
    Nav2 has seen, not about unexplored space.
    """
    step = grid.info.resolution
    cos_h, sin_h = math.cos(heading), math.sin(heading)
    lateral_steps = max(1, int(math.ceil(2.0 * half_width / step)))

    travelled = 0.0
    while travelled <= length:
        forward = start + travelled
        for k in range(lateral_steps + 1):
            side = -half_width + k * (2.0 * half_width / lateral_steps)
            px = x + forward * cos_h - side * sin_h
            py = y + forward * sin_h + side * cos_h
            if cost_at(grid, px, py) == LETHAL_COST:
                return travelled
        travelled += step

    return None


def free_distance_ahead(grid, x, y, yaw, look_ahead):
    """Free space in front of the robot's nose, in meters, up to look_ahead.
    None if nothing is in the way within look_ahead."""
    return first_obstacle_along(grid, x, y, yaw, ROBOT_FRONT_M, look_ahead)


def straight_line_clear(grid, x, y, heading, distance):
    """True if the robot could drive straight `distance` along `heading`
    without its body touching a known obstacle."""
    return first_obstacle_along(
        grid, x, y, heading, ROBOT_HALF_WIDTH_M, distance
    ) is None


def angle_difference(a, b):
    """Smallest absolute difference between two angles, in radians."""
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))
