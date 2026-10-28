"""Short backward and sideways moves when the forward path is blocked.

The checks take a laser scan and a costmap grid and do not touch ROS, so
tests can feed them handmade data. The scan is the live sensor. The grid is
Nav2's local obstacle layer, built from that scan. Neither one is the
accumulated 3D point map.
"""

import math

from go2_auto_mapper.obstacle_sensing import LETHAL_COST, cost_at

# nav2_params.yaml footprint: nose 0.36 m ahead of base_link, tail 0.45 m
# behind, body 0.40 m wide. The margin is added on every side of the swept
# body, so a corridor that only clears the outline still counts as too tight.
ROBOT_FRONT_M = 0.36
ROBOT_REAR_M = 0.45
ROBOT_HALF_WIDTH_M = 0.20
SAFETY_MARGIN_M = 0.05

RECOVERY_SPEED_MPS = 0.10
RECOVERY_TIME_LIMIT_SEC = 8.0
# /scan arrives at about 4 Hz. Older than this, the corridor is stale.
SCAN_MAX_AGE_SEC = 0.75
# A scan stamped further in the future than this points at a clock problem,
# so it says nothing about the present.
FUTURE_STAMP_TOLERANCE_SEC = 0.1
# Sensing required to release a stop must also be current, not merely newer
# than the stop: a source that sent one message and died is not updating.
# Slowest required source is the SLAM map (1 s), hence the margin.
SENSING_MAX_AGE_SEC = 3.0
# Long enough for a costmap update (2 Hz) and, when required, a SLAM map (1 s).
REFRESH_TIMEOUT_SEC = 12.0
# mapper_params_online_async.yaml minimum_travel_distance. A shorter move
# does not make slam_toolbox publish a new /map.
SLAM_MIN_TRAVEL_M = 0.2


def swept_bounds(dx, dy):
    """Robot-frame rectangle the padded body passes through for a translation
    of (dx, dy) meters. Orientation stays fixed."""

    return (
        -(ROBOT_REAR_M + SAFETY_MARGIN_M) + min(0.0, dx),
        ROBOT_FRONT_M + SAFETY_MARGIN_M + max(0.0, dx),
        -(ROBOT_HALF_WIDTH_M + SAFETY_MARGIN_M) + min(0.0, dy),
        ROBOT_HALF_WIDTH_M + SAFETY_MARGIN_M + max(0.0, dy),
    )


def _ahead_of_move(x, y, dx, dy):
    """True for robot-frame points level with or ahead of base_link along
    the move (dx, dy).

    The checks below look only at these. The obstacle that triggered the
    recovery sits on the other side, often within the inscribed radius of
    the nose; counting it made every move away from it look blocked, so the
    robot latched stopped next to the first close wall (found in go2_sim).
    Translating away from a point can never bring the body closer to it.
    """

    return x * dx + y * dy >= 0.0


def scan_covers_move(scan, dx, dy):
    """True when every sample of the swept body on the side it moves toward
    has a finite scan range past it.

    Missing rays, NaNs and infinite returns do not prove the space is clear.
    """

    if scan.angle_increment <= 0:
        return False
    count = len(scan.ranges)
    # pointcloud_to_laserscan's default scan (robot.launch.py) is 360 rays
    # from -pi: straight behind (+pi) is ray 0 again, not ray 360. Without
    # the wrap every backward move looked unmeasured (found in go2_sim).
    full_circle = count * scan.angle_increment >= 2 * math.pi - 1e-6
    x0, x1, y0, y1 = swept_bounds(dx, dy)
    step = 0.025
    for ix in range(math.floor(x0 / step), math.ceil(x1 / step) + 1):
        for iy in range(math.floor(y0 / step), math.ceil(y1 / step) + 1):
            x, y = ix * step, iy * step
            radius = math.hypot(x, y)
            if radius < scan.range_min or not _ahead_of_move(x, y, dx, dy):
                continue
            angle = math.atan2(y, x)
            index = round((angle - scan.angle_min) / scan.angle_increment)
            if full_circle:
                index %= count
            if not 0 <= index < count:
                return False
            value = scan.ranges[index]
            if not math.isfinite(value) or not scan.range_min <= value <= scan.range_max:
                return False
            if value < radius + SAFETY_MARGIN_M:
                return False
    return True


def swept_clear(grid, x, y, yaw, dx, dy):
    """True when the swept, padded footprint contains no obstacle on the
    side it moves toward (see _ahead_of_move).

    Unknown cells, cells off the grid and lethal (100) cells are unsafe.
    Inflation, including inscribed (99), is allowed: inscribed means "the
    robot's centre here would touch", which the full padded footprint already
    accounts for. Counting it too doubled the body width, so with a wall
    0.2 m beside the body (where Nav2 often leaves Oak) no move was ever
    clear (found in go2_sim). Nav2's own footprint checks use lethal only.
    """

    c, s = math.cos(yaw), math.sin(yaw)
    bounds = swept_bounds(dx, dy)
    corners = [
        (x + a * c - b * s, y + a * s + b * c)
        for a in bounds[:2] for b in bounds[2:]
    ]
    res = grid.info.resolution
    ox, oy = grid.info.origin.position.x, grid.info.origin.position.y
    pad = res * math.sqrt(2) / 2
    gx0 = math.floor((min(p[0] for p in corners) - ox) / res) - 1
    gx1 = math.ceil((max(p[0] for p in corners) - ox) / res) + 1
    gy0 = math.floor((min(p[1] for p in corners) - oy) / res) - 1
    gy1 = math.ceil((max(p[1] for p in corners) - oy) / res) + 1
    for gx in range(gx0, gx1):
        for gy in range(gy0, gy1):
            wx, wy = ox + (gx + 0.5) * res, oy + (gy + 0.5) * res
            along = (wx - x) * c + (wy - y) * s
            across = -(wx - x) * s + (wy - y) * c
            inside = (
                bounds[0] - pad <= along <= bounds[1] + pad
                and bounds[2] - pad <= across <= bounds[3] + pad
            )
            if not inside or not _ahead_of_move(along, across, dx, dy):
                continue
            cost = cost_at(grid, wx, wy)
            if cost is None or cost < 0 or cost >= LETHAL_COST:
                return False
    return True


def choose_recovery(grid, x, y, yaw, distance, scan):
    """The rear or side translation with the most extra free travel.

    Returns (dx, dy) in the robot frame, or None when no corridor is both
    clear on the costmap and covered by a finite scan. A missing scan is
    not treated as a clear one.
    """

    if scan is None:
        return None
    candidates = [(-distance, 0.0), (0.0, distance), (0.0, -distance)]
    safe = [
        move for move in candidates
        if swept_clear(grid, x, y, yaw, *move)
        and scan_covers_move(scan, *move)
    ]
    if not safe:
        return None
    return max(safe, key=lambda move: sum(
        swept_clear(grid, x, y, yaw, move[0] * scale, move[1] * scale)
        for scale in (1.5, 2.0, 3.0)
    ))


def scan_age(received_mono, now_mono, stamp_sec, ros_now_sec):
    """Age of a scan for safety decisions, in seconds, or None if unusable.

    The larger of the time since it was taken (header stamp against ROS
    time) and the time since it arrived (monotonic). Arrival alone isn't
    enough: a scan taken 2 s ago and delivered late by a transport or
    executor backlog would look brand new. A stamp in the future (beyond
    FUTURE_STAMP_TOLERANCE_SEC) proves nothing about the present.
    """

    acquired = ros_now_sec - stamp_sec
    if acquired < -FUTURE_STAMP_TOLERANCE_SEC:
        return None
    return max(now_mono - received_mono, acquired)


def assess_recovery(grid, x, y, yaw, distance, scan, scan_age, max_age):
    """(move, reason). `move` is None when the robot should stay stopped.

    `scan_age` is the scan's age in seconds (see scan_age()). Older than
    `max_age`, or missing, is stale and therefore unsafe.
    """

    if scan is None or scan_age is None or scan_age > max_age:
        return None, 'stale or missing laser scan'
    move = choose_recovery(grid, x, y, yaw, distance, scan)
    if move is None:
        return None, (
            'no safe direction: rear and both sides lack clearance '
            'for the footprint plus margin'
        )
    return move, ''


def latch_decision(forward_blocked, move):
    """What a latched stop should do with one new fresh scan.

    `forward_blocked` is whether something is still within the stop distance
    ahead. `move` is a recovery translation, or None when no direction is
    safe. Returns 'resume' (the block is gone), 'move' (a direction opened),
    or 'hold' (stay stopped).
    """

    if not forward_blocked:
        return 'resume'
    if move is None:
        return 'hold'
    return 'move'


def refresh_status(now, stop_time, timeout, stamps, need_map):
    """Whether sensing has updated since the robot stopped.

    `stamps` maps a source to its ROS timestamp in seconds, or None.
    scan, cloud, costmap, odom_tf and map_tf are always required, and each
    must be strictly newer than `stop_time`. slam_map is required only when
    `need_map` is set.

    `cloud` is the live /point_cloud2, not the accumulated 3D map. `costmap`
    is Nav2's obstacle layer (it raytraces /scan, which clears obstacles that
    moved or disappeared). `slam_map` is the SLAM occupancy grid. This does
    not ask anyone to reset the accumulated cloud.

    Returns (ready, reason). `reason` is empty when ready, a waiting note
    while the timeout still has time, and a timed-out note after that.
    """

    needed = ['scan', 'cloud', 'costmap', 'odom_tf', 'map_tf']
    if need_map:
        needed.append('slam_map')
    missing = [
        name for name in needed
        if stamps.get(name) is None or stamps[name] <= stop_time
    ]
    if not missing:
        return True, ''
    detail = ', '.join(missing)
    if now - stop_time > timeout:
        return False, f'timed out waiting for fresh {detail}'
    return False, f'waiting for fresh {detail}'


def stale_sources(now_ros, now_mono, sources, max_age):
    """Names of sources that aren't current.

    `sources` maps a name to (arrival in monotonic seconds or None, stamp in
    ROS seconds or None). A source is stale when it has no stamp, its stamp
    is more than `max_age` old, or (where the arrival time is known) it
    arrived more than `max_age` ago. Future stamps are not stale here:
    slam_toolbox stamps map->odom ahead on purpose (transform_timeout).
    """

    stale = []
    for name, (arrived, stamp) in sources.items():
        if (
            stamp is None
            or now_ros - stamp > max_age
            or (arrived is not None and now_mono - arrived > max_age)
        ):
            stale.append(name)
    return stale
