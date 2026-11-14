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
# a single recovery move in 4 runs. Aborting is safe: Oak just stops.
DRIFT_YAW_RAD = 0.10
DRIFT_ACROSS_M = 0.05
_YAW_SAMPLES = 7

# A leaving move (see leaving_ok): taken only when no normal move is safe,
# e.g. Oak wedged with a front corner 1 cm from a wall Nav2 drove it along,
# or with a wall cutting into a rear corner of the footprint (found in
# go2_sim). It may pass walls closer than the margin, as long as it brings
# nothing closer and covers no part of the footprint (or the margin around
# it) that a wall wasn't already in, so it must stay very close to
# straight: aborted at these limits, a corner swings at most about 1 cm.
CLOSE_DRIFT_YAW_RAD = 0.02
CLOSE_DRIFT_ACROSS_M = 0.01
# leaving_ok: how much closer than it started, or than the wall points
# already were close by, a scan return may get during a leaving move (the
# tight drift above). On the costmap it is half a cell diagonal instead
# (3.5 cm at 5 cm cells): a cell center is up to that far from the wall
# surface in it, the same allowance swept_clear pads cell centers with.
LEAVING_TOLERANCE_SCAN_M = 0.015
# "Close by" for leaving_ok: a point may get as deep as the wall points
# that were within this distance of where it ends up.
LEAVING_LOCAL_M = 0.10
# Deeper inside the bare footprint than this is not a wall brushing the
# outline (Nav2's footprint is a little larger than Oak's body), so no
# leaving move is allowed.
LEAVING_MAX_DEPTH_M = 0.12
_LEAVING_STEP_M = 0.02


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


def move_envelope(dx, dy, start=0.0):
    """Polygons (robot frame at the start pose) that together hold every
    place the padded body can be during the move (dx, dy), from `start`
    meters along it to the end, while the heading stays within
    DRIFT_YAW_RAD and the position within DRIFT_ACROSS_M of the line.

    The whole body is covered, both halves: Oak can turn while it moves,
    so the half facing away from the move can swing into a wall beside it.
    Returns (polygons, pad): `pad` covers the gaps between yaw samples.
    """

    distance = math.hypot(dx, dy)
    ux, uy = (dx / distance, dy / distance) if distance > 0 else (1.0, 0.0)
    nx, ny = -uy, ux
    front = ROBOT_FRONT_M + SAFETY_MARGIN_M
    rear = ROBOT_REAR_M + SAFETY_MARGIN_M
    half = ROBOT_HALF_WIDTH_M + SAFETY_MARGIN_M
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


def _trigger_exempt(x, y, dx, pad=0.0):
    """True for robot-frame points (start pose) that belong to the obstacle
    that stopped Oak, while the move backs away from it.

    The stop comes from an obstacle in front of the nose
    (free_distance_ahead), often within the padded nose, so counting it
    would refuse every move away from it and latch Oak stopped next to the
    first close wall (found in go2_sim). Only that obstacle is exempt: points
    past the nose, and only on a move with a backward part. Walls beside the
    body, and the nose obstacle on a sideways move, are always checked.
    """

    return dx < 0 and x >= ROBOT_FRONT_M - pad


def _depth(x, y):
    """How far a robot-frame point is inside the bare footprint, in meters;
    negative outside (minus its distance to the outline)."""

    inside = min(
        ROBOT_FRONT_M - x, x + ROBOT_REAR_M,
        ROBOT_HALF_WIDTH_M - y, y + ROBOT_HALF_WIDTH_M,
    )
    if inside >= 0:
        return inside
    gx = x - min(max(x, -ROBOT_REAR_M), ROBOT_FRONT_M)
    gy = y - min(max(y, -ROBOT_HALF_WIDTH_M), ROBOT_HALF_WIDTH_M)
    return -math.hypot(gx, gy)


def leaving_ok(points, dx, dy, tolerance, start=0.0):
    """True when the straight move (dx, dy) brings none of the obstacle
    points (robot frame at the start pose) closer to the bare footprint
    than it started, or deeper than the points already within
    LEAVING_LOCAL_M of where it ends up were, give or take `tolerance`.
    Only the SAFETY_MARGIN_M band around the footprint counts. Checked from
    `start` meters along it to the end, every _LEAVING_STEP_M.

    Sliding along a wall that cuts a corner passes: its points slide along
    the same line through the corner, as deep as their neighbors were.
    Moving away passes. Moving into a wall, or bringing another one into
    the footprint, doesn't. Points deeper than LEAVING_MAX_DEPTH_M refuse
    the move.
    """

    distance = math.hypot(dx, dy)
    if distance <= 0:
        return False
    ux, uy = dx / distance, dy / distance
    near = [(p, _depth(*p)) for p in points if _depth(*p) > -SAFETY_MARGIN_M]
    if any(depth > LEAVING_MAX_DEPTH_M for _, depth in near):
        return False
    steps = max(1, math.ceil((distance - start) / _LEAVING_STEP_M))
    for px, py in points:
        own = max(_depth(px, py), -SAFETY_MARGIN_M)
        for k in range(steps + 1):
            travel = start + (distance - start) * k / steps
            bx, by = px - travel * ux, py - travel * uy
            depth = _depth(bx, by)
            if depth <= own + tolerance:
                continue
            local = max((
                d for (nx, ny), d in near
                if math.hypot(bx - nx, by - ny) <= LEAVING_LOCAL_M
            ), default=-SAFETY_MARGIN_M)
            if depth > local + tolerance:
                return False
    return True


def scan_covers_move(scan, dx, dy, leaving=False):
    """True when no scan return falls inside the body's move envelope
    (move_envelope), except returns on the obstacle that stopped Oak
    (_trigger_exempt), and the new space the move enters is measured.

    Missing rays, NaNs and infinite returns do not prove the space is clear,
    so they refuse the move where they cross space ahead of base_link along
    the move. Behind it the body is already standing there, and the
    costmap check still covers it; the scan has a few empty rays (no
    return in its height band), and refusing on those blocked every
    backward move (found in go2_sim).

    `leaving` judges the returns inside the envelope with leaving_ok
    instead of refusing on any, for a leaving move run with the CLOSE_DRIFT
    limits, and leaves space with missing rays to the costmap check (which
    refuses moving toward unknown cells). In go2_sim the scan has a sector
    of empty rays beside the front (36 to 83 degrees on the left), and a
    leaving move out along a wall crosses it.
    """

    if scan.angle_increment <= 0:
        return False
    count = len(scan.ranges)
    # pointcloud_to_laserscan's default scan (robot.launch.py) is 360 rays
    # from -pi: straight behind (+pi) is ray 0 again, not ray 360. Without
    # the wrap every backward move looked unmeasured (found in go2_sim).
    full_circle = count * scan.angle_increment >= 2 * math.pi - 1e-6
    polygons, pad = move_envelope(dx, dy)
    xs = [p[0] for poly in polygons for p in poly]
    ys = [p[1] for poly in polygons for p in poly]
    step = 0.025
    hits = {}
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
                if not leaving and x * dx + y * dy >= 0.0:
                    return False
                continue
            if value < radius + SAFETY_MARGIN_M:
                hit_x = value * math.cos(angle)
                hit_y = value * math.sin(angle)
                if _trigger_exempt(hit_x, hit_y, dx):
                    continue
                if not leaving:
                    return False
                hits[index] = (hit_x, hit_y)
    return not hits or leaving_ok(
        list(hits.values()), dx, dy, LEAVING_TOLERANCE_SCAN_M
    )


def swept_clear(grid, x, y, yaw, dx, dy, start=0.0, leaving=False):
    """True when no wall cell lies in the body's move envelope
    (move_envelope) for the move (dx, dy) from the pose (x, y, yaw), counted
    from `start` meters along it. Mid-move, pass the start pose and the
    progress so far: the envelope is anchored where the move began.

    Unknown cells, cells off the grid and lethal (100) cells are unsafe,
    except the obstacle that stopped Oak (_trigger_exempt). Inflation,
    including inscribed (99), is allowed: the padded body plus the drift
    envelope already keeps the clearance. Counting it too doubled the body
    width, so no move was ever clear (found in go2_sim). Nav2's own
    footprint checks use lethal only.

    `leaving` judges the lethal cells in the envelope with leaving_ok
    instead of refusing on any, for a leaving move run with the CLOSE_DRIFT
    limits, and unknown cells the same way, separately: a move may not
    bring the body closer to unknown space either. (Unknown cells behind a
    wall that cuts into a corner are inside the padded envelope; refusing
    on them kept Oak stopped in go2_sim.) Off-grid cells still refuse it.
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
    walls, unknown = [], []
    for gx in range(gx0, gx1):
        for gy in range(gy0, gy1):
            wx, wy = ox + (gx + 0.5) * res, oy + (gy + 0.5) * res
            along = (wx - x) * c + (wy - y) * s
            across = -(wx - x) * s + (wy - y) * c
            if not _inside(polygons, along, across, pad):
                continue
            cost = cost_at(grid, wx, wy)
            if cost is None or (cost < 0 and not leaving):
                return False
            if cost < 0:
                unknown.append((along, across))
                continue
            if cost >= LETHAL_COST and not _trigger_exempt(
                along, across, dx, res / 2
            ):
                if not leaving:
                    return False
                walls.append((along, across))
    return all(
        not points or leaving_ok(points, dx, dy, res * math.sqrt(2) / 2, start)
        for points in (walls, unknown)
    )


def recovery_candidates(distance, leaving=False):
    """The rear and side translations recovery may take, robot frame.
    Leaving moves may also go diagonally back: out along a wall that cuts
    a rear corner at an angle (found in go2_sim)."""

    moves = [(-distance, 0.0), (0.0, distance), (0.0, -distance)]
    if leaving:
        d = distance / math.sqrt(2)
        moves += [(-d, d), (-d, -d)]
    return moves


def translation_open(grid, x, y, yaw, distance):
    """True while at least one normal recovery translation is clear on the
    costmap.

    Costmap only, for deciding when to stop (see watch_ahead): a move
    still needs choose_recovery, scan included, before it runs.
    """

    return any(
        swept_clear(grid, x, y, yaw, *move)
        for move in recovery_candidates(distance)
    )


def drift_limits(leaving):
    """(yaw, across) abort limits for a normal or a leaving move."""

    if leaving:
        return CLOSE_DRIFT_YAW_RAD, CLOSE_DRIFT_ACROSS_M
    return DRIFT_YAW_RAD, DRIFT_ACROSS_M


def choose_recovery_mode(grid, x, y, yaw, distance, scan, modes=(False, True)):
    """(move, leaving): the rear or side translation with the most extra
    free travel, and whether it is a leaving move.

    Normal moves come first. Only when none is safe is a leaving move tried
    (judged by leaving_ok, with the back diagonals too); it must
    run with the CLOSE_DRIFT limits. (None, False) when no corridor is both
    clear on the costmap and covered by a finite scan. A missing scan is
    not treated as a clear one. `modes` limits which kinds are tried.
    """

    if scan is None:
        return None, False
    for leaving in modes:
        safe = [
            move for move in recovery_candidates(distance, leaving)
            if swept_clear(grid, x, y, yaw, *move, leaving=leaving)
            and scan_covers_move(scan, *move, leaving=leaving)
        ]
        if safe:
            return max(safe, key=lambda move: sum(
                swept_clear(
                    grid, x, y, yaw, move[0] * scale, move[1] * scale,
                    leaving=leaving,
                )
                for scale in (1.5, 2.0, 3.0)
            )), leaving
    return None, False


def choose_recovery(grid, x, y, yaw, distance, scan):
    """The move from choose_recovery_mode, without the leaving flag."""

    return choose_recovery_mode(grid, x, y, yaw, distance, scan)[0]


# Turning in place when every short translation is refused, e.g. Oak turned
# crosswise in a narrow passage: the rear and the side moves are boxed in by
# the walls, but turning back along the passage is how it got there (found in
# go2_sim, where Oak latched stopped for good). Nav2's Spin checks the
# footprint against the local costmap while it turns, so this only picks the
# heading. Smallest turns first: on equal room the smaller turn wins.
TURN_CANDIDATES_RAD = tuple(
    math.radians(d) for d in (45, -45, 90, -90, 135, -135, 180)
)
# Rays this far either side of a candidate heading must all be open.
TURN_WINDOW_RAD = math.radians(15.0)


def open_range(scan, heading, window=TURN_WINDOW_RAD):
    """Shortest valid return within `window` of `heading` (robot frame,
    radians), or None when no ray there has a valid return.

    Missing, NaN and infinite rays are skipped: they don't show the space is
    open, so a heading seen only through them is never picked.
    """

    if scan.angle_increment <= 0:
        return None
    count = len(scan.ranges)
    full_circle = count * scan.angle_increment >= 2 * math.pi - 1e-6
    steps = int(math.ceil(window / scan.angle_increment))
    shortest = None
    for k in range(-steps, steps + 1):
        angle = math.atan2(
            math.sin(heading + k * scan.angle_increment),
            math.cos(heading + k * scan.angle_increment),
        )
        index = round((angle - scan.angle_min) / scan.angle_increment)
        if full_circle:
            index %= count
        if not 0 <= index < count:
            continue
        value = scan.ranges[index]
        if not math.isfinite(value) or not scan.range_min <= value <= scan.range_max:
            continue
        if shortest is None or value < shortest:
            shortest = value
    return shortest


def choose_turn(scan, min_range, exclude_sign=0):
    """Turn (radians, positive is left) toward the candidate heading with the
    most room on the scan, or None when none has at least `min_range` meters.

    `exclude_sign` (1 or -1) skips turns that way, e.g. after Nav2 aborted a
    turn that way. A missing scan gives None.
    """

    if scan is None:
        return None
    best, best_room = None, None
    for turn in TURN_CANDIDATES_RAD:
        if exclude_sign and math.copysign(1.0, turn) == exclude_sign:
            continue
        room = open_range(scan, turn)
        if room is None or room < min_range:
            continue
        if best_room is None or room > best_room:
            best, best_room = turn, room
    return best


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
    """(move, reason, leaving). `move` is None when the robot should stay
    stopped; `leaving` says it is a leaving move (choose_recovery_mode).

    `scan_age` is the scan's age in seconds (see scan_age()). Older than
    `max_age`, or missing, is stale and therefore unsafe.
    """

    if scan is None or scan_age is None or scan_age > max_age:
        return None, 'stale or missing laser scan', False
    move, leaving = choose_recovery_mode(grid, x, y, yaw, distance, scan)
    if move is None:
        return None, (
            'no safe direction: rear and both sides lack clearance '
            'for the footprint plus margin'
        ), False
    return move, '', leaving


def assess_recovery_steps(grid, x, y, yaw, distances, scan, scan_age, max_age):
    """(move, reason, leaving): the first safe move from longest to shortest
    distance, like assess_recovery but trying each distance in turn.

    Every candidate goes through the same full-body costmap and live-scan
    checks as the original recovery. A shorter move is not permission to
    cross an obstacle; it is only useful when the longer endpoint does not
    fit in the measured free corridor. Normal moves at every distance come
    before any leaving move (choose_recovery_mode).
    """

    if scan is None or scan_age is None or scan_age > max_age:
        return None, 'stale or missing laser scan', False
    for leaving in (False, True):
        for distance in distances:
            move, _ = choose_recovery_mode(
                grid, x, y, yaw, float(distance), scan, modes=(leaving,)
            )
            if move is not None:
                return move, '', leaving
    detail = ', '.join(f'{float(distance):.2f}' for distance in distances)
    return None, (
        'no safe direction: rear and both sides lack clearance '
        f'for the footprint plus margin at {detail} m'
    ), False


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
