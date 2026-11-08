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
# A wedged Oak may have a safe corridor that is shorter than the normal
# 0.3 m recovery. Try bounded shorter moves rather than releasing the latch
# or weakening the footprint and sensor-clearance checks.
RECOVERY_FALLBACK_DISTANCES_M = (0.15, 0.08)


# How far a recovery move may wander before tick_recovery aborts it:
# heading drift from the start yaw, and sideways drift off the commanded
# line. The clearance checks cover the body anywhere inside that envelope,
# so an abort always comes before the body leaves checked space. Kept tight
# because the envelope grows with them: at 0.10 m and 0.15 rad a wall
# 0.17 m beside the body refused every move, and in go2_sim Oak never made
# a single recovery move in 4 runs. At 0.10 rad and 0.05 m a wall 0.10 m
# beside the body still refused a straight-back move with 1.5 m free behind
# (found from go2_sim recordings). Aborting is safe: Oak just stops.
DRIFT_YAW_RAD = 0.06
DRIFT_ACROSS_M = 0.03
_YAW_SAMPLES = 7
# A short run of rays with no return is normal: in go2_sim 17 to 43 of 360
# rays were empty at every freeze, in runs of 1 to 10 degrees behind Oak, and
# they alone refused a backward move with 2.5 m of free floor. A wider run
# could be something the lidar cannot see, such as glass, so it still refuses.
MAX_MISSING_GAP_RAD = math.radians(15.0)
# How far past the trailing edge a point must be for a sideways or diagonal
# move to ignore it: 0.49 m body radius times DRIFT_YAW_RAD, rounded up.
SWING_CLEAR_M = 0.04


def swept_bounds(dx, dy):
    """Robot-frame rectangle the padded body passes through for a translation
    of (dx, dy) meters. Orientation stays fixed."""

    return (
        -(ROBOT_REAR_M + SAFETY_MARGIN_M) + min(0.0, dx),
        ROBOT_FRONT_M + SAFETY_MARGIN_M + max(0.0, dx),
        -(ROBOT_HALF_WIDTH_M + SAFETY_MARGIN_M) + min(0.0, dy),
        ROBOT_HALF_WIDTH_M + SAFETY_MARGIN_M + max(0.0, dy),
    )


def _hull(points):
    """Convex hull, counter-clockwise (monotone chain)."""

    points = sorted(set(points))

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in points:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(points):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def move_envelope(dx, dy, start=0.0, margin=SAFETY_MARGIN_M):
    """Polygons (robot frame at the start pose) that together hold every
    place the padded body can be during the move (dx, dy), from `start`
    meters along it to the end, while the heading stays within
    DRIFT_YAW_RAD and the position within DRIFT_ACROSS_M of the line.

    The whole body is covered, both halves: Oak can turn while it moves,
    so the half facing away from the move can swing into a wall beside it.
    Returns (polygons, pad): `pad` covers the gaps between yaw samples.
    `margin` is the padding around the body outline.
    """

    distance = math.hypot(dx, dy)
    ux, uy = (dx / distance, dy / distance) if distance > 0 else (1.0, 0.0)
    nx, ny = -uy, ux
    front = ROBOT_FRONT_M + margin
    rear = ROBOT_REAR_M + margin
    half = ROBOT_HALF_WIDTH_M + margin
    corners = [(a, b) for a in (-rear, front) for b in (-half, half)]
    offsets = [
        (s * ux + side * nx, s * uy + side * ny)
        for s in (min(start, distance), distance)
        for side in (-DRIFT_ACROSS_M, DRIFT_ACROSS_M)
    ]
    polygons = []
    for k in range(_YAW_SAMPLES):
        theta = -DRIFT_YAW_RAD + 2 * DRIFT_YAW_RAD * k / (_YAW_SAMPLES - 1)
        c, s = math.cos(theta), math.sin(theta)
        body = [(a * c - b * s, a * s + b * c) for a, b in corners]
        polygons.append(_hull([
            (bx + ox, by + oy) for bx, by in body for ox, oy in offsets
        ]))
    step = 2 * DRIFT_YAW_RAD / (_YAW_SAMPLES - 1)
    pad = math.hypot(rear, half) * math.sin(step / 2)
    return polygons, pad


def _inside(polygons, x, y, pad):
    """True when (x, y) is within `pad` of any of the convex polygons."""

    for poly in polygons:
        for i, (x0, y0) in enumerate(poly):
            x1, y1 = poly[(i + 1) % len(poly)]
            length = math.hypot(x1 - x0, y1 - y0)
            if (x1 - x0) * (y - y0) - (y1 - y0) * (x - x0) < -pad * length:
                break
        else:
            return True
    return False


def _behind_move(x, y, dx, dy, pad=0.0):
    """True for robot-frame points (start pose) behind the body's trailing
    edge: the move only takes the body further from them.

    Oak often stops already close to something, so counting it would refuse
    every move away from it and latch Oak next to it (found in go2_sim, for
    the nose while backing up and for a wall beside the body while sidestepping
    away). Points level with the body, or ahead of its trailing edge, are
    always checked. Except straight back from the nose, a point must also
    be SWING_CLEAR_M past the edge: a small turn at the start of the move
    swings the body toward what it is leaving.
    """

    distance = math.hypot(dx, dy)
    if distance <= 0.0:
        return False
    ux, uy = dx / distance, dy / distance
    trailing = min(
        a * ux + b * uy
        for a in (-ROBOT_REAR_M, ROBOT_FRONT_M)
        for b in (-ROBOT_HALF_WIDTH_M, ROBOT_HALF_WIDTH_M)
    )
    if dx < 0.0 and dy == 0.0:
        return x * ux + y * uy <= trailing + pad
    return x * ux + y * uy <= trailing - SWING_CLEAR_M


def _margin_exempt(x, y, dx, dy, pad=0.0, slack=0.0):
    """True for a robot-frame point that is already inside the padding where
    Oak stands now, and that the bare body cannot reach during the move.

    Without this, a wall 0.10 to 0.15 m beside the body refuses every move,
    of any length, including the ones that slide straight along it (found
    from go2_sim recordings). The padding still stops Oak moving toward new
    obstacles; it just cannot pin Oak against one it is already next to.
    `slack` widens only the "already inside the padding" test, for callers
    whose own check reaches past the padding (the scan's extra margin, the
    costmap's cell size).
    """

    distance = math.hypot(dx, dy)
    if distance <= 0.0:
        return False
    scale = 1e-6 / distance
    here, here_pad = move_envelope(dx * scale, dy * scale)
    if not _inside(here, x, y, here_pad + pad + slack):
        return False
    bare, bare_pad = move_envelope(dx, dy, margin=0.0)
    return not _inside(bare, x, y, bare_pad + pad)


def _wide_gap_rays(scan, full_circle):
    """Indices of no-return rays in a run wider than MAX_MISSING_GAP_RAD."""

    count = len(scan.ranges)
    missing = [
        not math.isfinite(value) or not scan.range_min <= value <= scan.range_max
        for value in scan.ranges
    ]
    if all(missing):
        return set(range(count))
    limit = MAX_MISSING_GAP_RAD / scan.angle_increment
    # Start the walk on a valid ray, so a run across the wrap stays whole.
    begin = missing.index(False) if full_circle else 0
    wide, run = set(), []
    for offset in range(count + 1):
        index = (begin + offset) % count
        if offset < count and missing[index]:
            run.append(index)
            continue
        if len(run) > limit:
            wide.update(run)
        run = []
    return wide


def scan_covers_move(scan, dx, dy):
    """True when no scan return falls inside the body's move envelope
    (move_envelope), except returns behind the move
    (_behind_move) or already in the padding (_margin_exempt), and the new
    space the move enters is measured.

    Missing rays, NaNs and infinite returns do not prove the space is clear,
    so a wide run of them (MAX_MISSING_GAP_RAD) refuses the move where it
    crosses space ahead of base_link along the move. Behind it the body is
    already standing there. A narrow run is a normal sparse scan: the rays
    either side of it measured the space and the costmap check still covers
    it. Refusing on those blocked every backward move (found in go2_sim).
    """

    if scan.angle_increment <= 0:
        return False
    count = len(scan.ranges)
    # pointcloud_to_laserscan's default scan (robot.launch.py) is 360 rays
    # from -pi: straight behind (+pi) is ray 0 again, not ray 360. Without
    # the wrap every backward move looked unmeasured (found in go2_sim).
    full_circle = count * scan.angle_increment >= 2 * math.pi - 1e-6
    wide_gaps = _wide_gap_rays(scan, full_circle)
    distance = math.hypot(dx, dy) or 1.0
    ux, uy = dx / distance, dy / distance
    # The leading edge of the padded body along the move. Only space past it
    # is new; what the body stands beside is the costmap check's job.
    leading = max(
        a * ux + b * uy
        for a in (-(ROBOT_REAR_M + SAFETY_MARGIN_M), ROBOT_FRONT_M + SAFETY_MARGIN_M)
        for b in (-(ROBOT_HALF_WIDTH_M + SAFETY_MARGIN_M),
                  ROBOT_HALF_WIDTH_M + SAFETY_MARGIN_M)
    )
    polygons, pad = move_envelope(dx, dy)
    xs = [p[0] for poly in polygons for p in poly]
    ys = [p[1] for poly in polygons for p in poly]
    step = 0.025
    for ix in range(math.floor(min(xs) / step), math.ceil(max(xs) / step) + 1):
        for iy in range(math.floor(min(ys) / step), math.ceil(max(ys) / step) + 1):
            x, y = ix * step, iy * step
            radius = math.hypot(x, y)
            if radius < scan.range_min or not _inside(polygons, x, y, pad):
                continue
            angle = math.atan2(y, x)
            index = round((angle - scan.angle_min) / scan.angle_increment)
            if full_circle:
                index %= count
            if not 0 <= index < count:
                return False
            value = scan.ranges[index]
            if not math.isfinite(value) or not scan.range_min <= value <= scan.range_max:
                if index in wide_gaps and x * ux + y * uy > leading:
                    return False
                continue
            if value < radius + SAFETY_MARGIN_M:
                hit_x = value * math.cos(angle)
                hit_y = value * math.sin(angle)
                if not (_behind_move(hit_x, hit_y, dx, dy)
                        or _margin_exempt(hit_x, hit_y, dx, dy,
                                          slack=SAFETY_MARGIN_M)):
                    return False
    return True


def swept_clear(grid, x, y, yaw, dx, dy, start=0.0):
    """True when no wall cell lies in the body's move envelope
    (move_envelope) for the move (dx, dy) from the pose (x, y, yaw), counted
    from `start` meters along it. Mid-move, pass the start pose and the
    progress so far: the envelope is anchored where the move began.

    Unknown cells, cells off the grid and lethal (100) cells are unsafe,
    except points behind the move (_behind_move) and points already inside
    the padding that the bare body cannot reach (_margin_exempt). Inflation,
    including inscribed (99), is allowed: the padded body plus the drift
    envelope already keeps the clearance. Counting it too doubled the body
    width, so no move was ever clear (found in go2_sim). Nav2's own
    footprint checks use lethal only.
    """

    c, s = math.cos(yaw), math.sin(yaw)
    polygons, pad = move_envelope(dx, dy, start)
    local = [p for poly in polygons for p in poly]
    corners = [(x + a * c - b * s, y + a * s + b * c) for a, b in local]
    res = grid.info.resolution
    ox, oy = grid.info.origin.position.x, grid.info.origin.position.y
    pad += res * math.sqrt(2) / 2
    gx0 = math.floor((min(p[0] for p in corners) - ox) / res) - 1
    gx1 = math.ceil((max(p[0] for p in corners) - ox) / res) + 1
    gy0 = math.floor((min(p[1] for p in corners) - oy) / res) - 1
    gy1 = math.ceil((max(p[1] for p in corners) - oy) / res) + 1
    for gx in range(gx0, gx1):
        for gy in range(gy0, gy1):
            wx, wy = ox + (gx + 0.5) * res, oy + (gy + 0.5) * res
            along = (wx - x) * c + (wy - y) * s
            across = -(wx - x) * s + (wy - y) * c
            if not _inside(polygons, along, across, pad):
                continue
            cost = cost_at(grid, wx, wy)
            if cost is None or cost < 0:
                return False
            if cost >= LETHAL_COST and not (
                _behind_move(along, across, dx, dy, res / 2)
                or _margin_exempt(along, across, dx, dy, res / 2, res)
            ):
                return False
    return True


def choose_recovery(grid, x, y, yaw, distance, scan):
    """The rear, side or rear-diagonal translation with the most extra free
    travel.

    Returns (dx, dy) in the robot frame, or None when no corridor is both
    clear on the costmap and covered by a finite scan. A missing scan is
    not treated as a clear one.
    """

    if scan is None:
        return None
    diagonal = distance * math.sqrt(0.5)
    candidates = [
        (-distance, 0.0), (0.0, distance), (0.0, -distance),
        (-diagonal, diagonal), (-diagonal, -diagonal),
    ]
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


def recovery_distances(requested, fallbacks=RECOVERY_FALLBACK_DISTANCES_M):
    """Unique recovery distances, longest first and never above requested.

    The configured distance remains the first choice. Fallbacks only make a
    move shorter; they cannot expand the requested motion or go below the
    existing 5 cm minimum.
    """

    maximum = min(max(float(requested), 0.05), 0.5)
    candidates = [maximum]
    candidates.extend(
        min(max(float(distance), 0.05), 0.5)
        for distance in fallbacks
        if float(distance) < maximum
    )
    return tuple(dict.fromkeys(candidates))


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


def assess_recovery_steps(grid, x, y, yaw, distances, scan, scan_age, max_age):
    """Choose the first safe move from longest to shortest distance.

    Every candidate goes through the same full-body costmap and live-scan
    checks as the original recovery. A shorter move is not permission to
    cross an obstacle; it is only useful when the longer endpoint does not
    fit in the measured free corridor.
    """

    if scan is None or scan_age is None or scan_age > max_age:
        return None, 'stale or missing laser scan'
    checked = []
    for distance in distances:
        checked.append(float(distance))
        move = choose_recovery(grid, x, y, yaw, float(distance), scan)
        if move is not None:
            return move, ''
    detail = ', '.join(f'{distance:.2f}' for distance in checked)
    return None, (
        'no safe direction: rear and both sides lack clearance '
        f'for the footprint plus margin at {detail} m'
    )


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
