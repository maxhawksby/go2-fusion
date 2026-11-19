"""Turn a lab rosbag into stop distance, command latency or object visibility."""

import argparse
from bisect import bisect_left, bisect_right
from dataclasses import dataclass, field
import math
import os
from statistics import median
from typing import List, NamedTuple, Optional

import numpy as np

from go2_scoreboard.replay import open_bag


# `ros2 topic pub --rate 10` sends a command every 0.1 s; a longer silence
# than this means the operator started a new repeat.
COMMAND_GAP_S = 1.0
# Oak has started moving once its pose is this far from where the command
# found it: above odometry jitter, and under 0.1 s of travel at lab speeds.
MOTION_START_DISTANCE_M = 0.02
MOTION_START_YAW_RAD = 0.05
# Gyro yaw rate that counts as turning: far below the 0.8 rad/s commanded,
# above the noise of a standing Oak. TODO(Max): check on the first real bag.
MOTION_START_GYRO_RADPS = 0.1
# Oak is at rest once its pose stays inside this radius and yaw band for the
# hold time. Tighter than a tape measure reads; the hold skips a mid-step pause.
REST_RADIUS_M = 0.01
REST_YAW_RAD = 0.02
REST_HOLD_S = 0.5
# /odom twist is always zero on Oak (#124), so speed is a pose difference.
# Differences over less than this span are mostly odometry noise.
SPEED_SPAN_S = 0.5

WINDOW_S = 5.0
# "Ahead" for the object check: the strip Oak would walk through.
CENTRE_HALF_WIDTH_M = 0.25
BOX_NEAR_M = 0.2
BOX_FAR_M = 1.2
# Height bands in base_link, which sits at body height: floor, low object, body and above.
BAND_EDGES_M = (-0.25, 0.0)
BAND_NAMES = ('below -0.25 m', '-0.25 to 0 m', 'above 0 m')

KIND_TOPICS = {
    'stop': ('/cmd_vel_joy', '/odom'),
    'turn': ('/cmd_vel_joy', '/odom', '/imu'),
    'object': ('/scan', '/point_cloud2', '/odom'),
}


class Command(NamedTuple):
    t: float
    linear: float
    angular: float


class Pose(NamedTuple):
    t: float
    x: float
    y: float
    yaw: float


class Gyro(NamedTuple):
    t: float
    yaw_rate: float


class Scan(NamedTuple):
    t: float
    angle_min: float
    angle_increment: float
    ranges: tuple


class Cloud(NamedTuple):
    t: float
    points: np.ndarray  # N x 3, metres, base_link


@dataclass(frozen=True)
class Repeat:
    """One command burst and what Oak did about it. None means not seen."""

    command_start_s: float
    command_end_s: float
    commanded_linear: float
    commanded_angular: float
    motion_start_s: Optional[float]
    gyro_motion_start_s: Optional[float]
    rest_s: Optional[float]
    distance_after_command_m: Optional[float]
    peak_speed_mps: Optional[float]
    peak_turn_rate_radps: Optional[float]


@dataclass(frozen=True)
class ObjectWindow:
    start_s: float
    scan_count: int
    nearest_m: Optional[float]
    median_nearest_m: Optional[float]
    cloud_count: int
    mean_points_by_band: Optional[tuple]


@dataclass
class LabSamples:
    commands: List[Command] = field(default_factory=list)
    poses: List[Pose] = field(default_factory=list)
    gyro: List[Gyro] = field(default_factory=list)
    scans: List[Scan] = field(default_factory=list)
    clouds: List[Cloud] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)


def _unwrap(poses):
    unwrapped = []
    previous = None
    for pose in poses:
        yaw = pose.yaw
        if previous is not None:
            yaw = previous + math.remainder(pose.yaw - previous, math.tau)
        unwrapped.append(pose._replace(yaw=yaw))
        previous = yaw
    return unwrapped


def _pose_at(poses, times, t):
    """Pose at time t, interpolated between the two samples around it."""
    index = bisect_left(times, t)
    if index == 0:
        return poses[0]._replace(t=t)
    if index == len(poses):
        return poses[-1]._replace(t=t)
    before, after = poses[index - 1], poses[index]
    share = (t - before.t) / (after.t - before.t)
    return Pose(t,
                before.x + share * (after.x - before.x),
                before.y + share * (after.y - before.y),
                before.yaw + share * (after.yaw - before.yaw))


def _distance(a, b):
    return math.hypot(b.x - a.x, b.y - a.y)


def _command_bursts(commands):
    bursts = []
    for command in commands:
        if command.linear == 0.0 and command.angular == 0.0:
            continue
        if bursts and command.t - bursts[-1][-1].t <= COMMAND_GAP_S:
            bursts[-1].append(command)
        else:
            bursts.append([command])
    return bursts


def _rest(candidates, limit_s):
    """First pose Oak then stays near for REST_HOLD_S, and the pose it settled at."""
    for index, start in enumerate(candidates):
        if start.t + REST_HOLD_S > limit_s:
            return None
        for later in candidates[index + 1:]:
            if (_distance(start, later) > REST_RADIUS_M
                    or abs(later.yaw - start.yaw) > REST_YAW_RAD):
                break
            if later.t >= start.t + REST_HOLD_S:
                return start, later
    return None


def _peak_rates(poses):
    """Fastest speed and turn rate over any span of at least SPEED_SPAN_S."""
    speed = turn = None
    earlier = 0
    for pose in poses:
        while earlier + 1 < len(poses) and poses[earlier + 1].t <= pose.t - SPEED_SPAN_S:
            earlier += 1
        span = pose.t - poses[earlier].t
        if span < SPEED_SPAN_S:
            continue
        speed = max(speed or 0.0, _distance(poses[earlier], pose) / span)
        turn = max(turn or 0.0, abs(pose.yaw - poses[earlier].yaw) / span)
    return speed, turn


def measure_repeats(commands, poses, gyro=()):
    """Split the commands into repeats and measure Oak's response to each."""
    poses = _unwrap(poses)
    times = [pose.t for pose in poses]
    gyro_times = [sample.t for sample in gyro]
    bursts = _command_bursts(commands)
    repeats = []
    for number, burst in enumerate(bursts):
        start_s, end_s = burst[0].t, burst[-1].t
        limit_s = bursts[number + 1][0].t if number + 1 < len(bursts) else math.inf
        motion_start_s = rest_s = distance = speed = turn = None
        if poses:
            origin = _pose_at(poses, times, start_s)
            response = poses[bisect_right(times, start_s):bisect_left(times, limit_s)]
            motion_start_s = next(
                (pose.t for pose in response
                 if _distance(origin, pose) > MOTION_START_DISTANCE_M
                 or abs(pose.yaw - origin.yaw) > MOTION_START_YAW_RAD), None)
            at_command_end = _pose_at(poses, times, end_s)
            # A burst shorter than the latency ends before Oak moves: that is not rest.
            rest_from_s = max(end_s, motion_start_s or end_s)
            after = [pose for pose in [at_command_end] + response
                     if pose.t >= rest_from_s]
            rest = _rest(after, min(limit_s, times[-1]))
            if rest is not None:
                rest_s = rest[0].t
                # Measured to the settled pose, so the rest radius does not trim it.
                distance = _distance(at_command_end, rest[1])
            speed, turn = _peak_rates([origin] + response)
        window = gyro[bisect_left(gyro_times, start_s):bisect_left(gyro_times, limit_s)]
        gyro_motion_start_s = next(
            (sample.t for sample in window
             if abs(sample.yaw_rate) > MOTION_START_GYRO_RADPS), None)
        repeats.append(Repeat(
            command_start_s=start_s,
            command_end_s=end_s,
            commanded_linear=max((c.linear for c in burst), key=abs),
            commanded_angular=max((c.angular for c in burst), key=abs),
            motion_start_s=motion_start_s,
            gyro_motion_start_s=gyro_motion_start_s,
            rest_s=rest_s,
            distance_after_command_m=distance,
            peak_speed_mps=speed,
            peak_turn_rate_radps=turn,
        ))
    return repeats


def nearest_ahead(scan):
    """Forward distance of the closest return in the strip ahead, or None."""
    ranges = np.asarray(scan.ranges, dtype=float)
    angles = scan.angle_min + scan.angle_increment * np.arange(len(ranges))
    hit = np.isfinite(ranges) & (ranges > 0.0)
    forward = ranges[hit] * np.cos(angles[hit])
    sideways = ranges[hit] * np.sin(angles[hit])
    ahead = forward[(forward > 0.0) & (np.abs(sideways) <= CENTRE_HALF_WIDTH_M)]
    return float(ahead.min()) if ahead.size else None


def box_counts(points):
    """Points in the box ahead of Oak, counted per height band."""
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    box = points[(points[:, 0] >= BOX_NEAR_M) & (points[:, 0] <= BOX_FAR_M)
                 & (np.abs(points[:, 1]) <= CENTRE_HALF_WIDTH_M)]
    bands = np.searchsorted(BAND_EDGES_M, box[:, 2], side='right')
    return tuple(int(count) for count in np.bincount(bands, minlength=len(BAND_NAMES)))


def to_base_link(points, position, quaternion):
    """Move odom-frame points into base_link given Oak's pose in odom."""
    x, y, z, w = quaternion
    rotation = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    return (np.asarray(points, dtype=float).reshape(-1, 3) - np.asarray(position)) @ rotation


def object_windows(scans, clouds):
    """Summarise what the scan and the cloud saw ahead in each WINDOW_S window."""
    last = max([s.t for s in scans] + [c.t for c in clouds], default=None)
    if last is None:
        return []
    windows = []
    for index in range(int(last // WINDOW_S) + 1):
        start_s = index * WINDOW_S
        inside = [s for s in scans if start_s <= s.t < start_s + WINDOW_S]
        nearest = [d for d in map(nearest_ahead, inside) if d is not None]
        counts = [box_counts(c.points) for c in clouds
                  if start_s <= c.t < start_s + WINDOW_S]
        windows.append(ObjectWindow(
            start_s=start_s,
            scan_count=len(inside),
            nearest_m=min(nearest) if nearest else None,
            median_nearest_m=median(nearest) if nearest else None,
            cloud_count=len(counts),
            mean_points_by_band=tuple(np.mean(counts, axis=0)) if counts else None,
        ))
    return windows


def _cell(value, digits=2):
    return '-' if value is None else f'{value:.{digits}f}'


def _since(later, earlier):
    return None if later is None or earlier is None else later - earlier


def _table(header, rows):
    lines = ['| ' + ' | '.join(header) + ' |', '|' + ' --- |' * len(header)]
    lines += ['| ' + ' | '.join(row) + ' |' for row in rows]
    return lines


def _summary(label, values, unit):
    known = [value for value in values if value is not None]
    if not known:
        return f'- {label}: not measured.'
    worst = max(known)
    return (f'- {label}: median {median(known):.2f} {unit}, worst {worst:.2f} {unit} '
            f'({len(known)} of {len(values)} repeats).')


def stop_report(repeats):
    lines = ['# Lab measurement: stop distance', '']
    if not repeats:
        return '\n'.join(lines + ['No repeats found: no non-zero `/cmd_vel_joy` command.'])
    times = [_since(r.rest_s, r.command_end_s) for r in repeats]
    rows = [(str(number), f'{r.command_start_s:.1f} to {r.command_end_s:.1f}',
             _cell(r.commanded_linear), _cell(r.peak_speed_mps), _cell(time),
             _cell(r.distance_after_command_m))
            for number, (r, time) in enumerate(zip(repeats, times), 1)]
    lines += _table(('Repeat', 'Command (s)', 'Commanded (m/s)', 'Speed reached (m/s)',
                     'Time to rest (s)', 'Stop distance (m)'), rows)
    lines += ['',
              _summary('Time from last command to rest', times, 's'),
              _summary('Stop distance', [r.distance_after_command_m for r in repeats], 'm'),
              '',
              'Stop distance is the straight line from where Oak was at the last command '
              'to where it came to rest. Speed is the fastest pose change over '
              f'{SPEED_SPAN_S} s. Rest means the pose stayed within {REST_RADIUS_M} m '
              f'for {REST_HOLD_S} s. A dash means Oak never came to rest in the recording.']
    return '\n'.join(lines)


def turn_report(repeats):
    lines = ['# Lab measurement: command latency', '']
    if not repeats:
        return '\n'.join(lines + ['No repeats found: no non-zero `/cmd_vel_joy` command.'])
    gyro = [_since(r.gyro_motion_start_s, r.command_start_s) for r in repeats]
    odom = [_since(r.motion_start_s, r.command_start_s) for r in repeats]
    gap = [_since(o, g) for o, g in zip(odom, gyro)]
    rows = [(str(number), f'{r.command_start_s:.1f}', _cell(g), _cell(o), _cell(d),
             _cell(r.peak_turn_rate_radps), _cell(abs(r.commanded_angular)))
            for number, (r, g, o, d) in enumerate(zip(repeats, gyro, odom, gap), 1)]
    lines += _table(('Repeat', 'Command start (s)', 'Gyro delay (s)', 'Odom delay (s)',
                     'Odom minus gyro (s)', 'Turn rate reached (rad/s)',
                     'Commanded (rad/s)'), rows)
    lines += ['',
              _summary('Delay to first motion in `/imu` gyro', gyro, 's'),
              _summary('Delay to first motion in `/odom` pose', odom, 's'),
              _summary('Odom minus gyro', gap, 's'),
              _summary('Turn rate reached', [r.peak_turn_rate_radps for r in repeats],
                       'rad/s'),
              '']
    if any(value is not None for value in gyro):
        lines.append('Odom minus gyro near zero means Oak itself moves late. '
                     'A large value means the odometry reports late.')
    else:
        lines.append('No motion seen in `/imu`. Only the odom delay is known, so this '
                     'recording cannot say whether the delay is Oak or the odometry.')
    lines.append(f'Motion starts when the gyro passes {MOTION_START_GYRO_RADPS} rad/s '
                 f'or the pose yaw changes by {MOTION_START_YAW_RAD} rad. '
                 f'Turn rate is the fastest yaw change over {SPEED_SPAN_S} s.')
    return '\n'.join(lines)


def object_report(windows):
    lines = ['# Lab measurement: low object visibility', '']
    if not windows:
        return '\n'.join(lines + ['No `/scan` or `/point_cloud2` message found.'])
    rows = []
    for window in windows:
        bands = window.mean_points_by_band or (None,) * len(BAND_NAMES)
        rows.append((f'{window.start_s:.0f} to {window.start_s + WINDOW_S:.0f}',
                     str(window.scan_count), _cell(window.nearest_m),
                     _cell(window.median_nearest_m), str(window.cloud_count),
                     *(_cell(count, 1) for count in bands)))
    lines += _table(('Window (s)', 'Scans', 'Nearest scan return (m)',
                     'Median nearest (m)', 'Clouds',
                     *(f'Cloud points {name}' for name in BAND_NAMES)), rows)
    lines += ['',
              f'Scan columns: forward distance of the closest `/scan` return within '
              f'{CENTRE_HALF_WIDTH_M} m of the centre line ahead. Nearest is the closest '
              'in any scan of the window, median is the typical scan. A dash means no '
              'return ahead.',
              f'Cloud columns: mean points per cloud in the box {BOX_NEAR_M} to '
              f'{BOX_FAR_M} m ahead and {CENTRE_HALF_WIDTH_M} m either side, by height '
              'in `base_link`.']
    return '\n'.join(lines)


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _cloud_points(message):
    offsets = {item.name: item.offset for item in message.fields}
    layout = np.dtype({'names': ['x', 'y', 'z'], 'formats': ['<f4'] * 3,
                       'offsets': [offsets['x'], offsets['y'], offsets['z']],
                       'itemsize': message.point_step})
    raw = np.frombuffer(bytes(message.data), dtype=layout,
                        count=message.width * message.height)
    points = np.stack([raw['x'], raw['y'], raw['z']], axis=1).astype(float)
    return points[np.isfinite(points).all(axis=1)]


def read_lab_bag(bag_path, topics):
    """Read the named topics into samples timed in seconds from the first one.

    Times are when the recorder received each message, not header stamps:
    it is the one clock every topic shares, and Twist commands carry no stamp.
    """
    reader, message_types, warnings = open_bag(bag_path)
    import rosbag2_py
    from rclpy.serialization import deserialize_message

    samples = LabSamples(warnings=warnings)
    for topic in topics:
        if topic not in message_types:
            samples.warnings.append(f'{topic} is not in this recording')
    wanted = [topic for topic in topics if topic in message_types]
    if not wanted:
        return samples
    reader.set_filter(rosbag2_py.StorageFilter(topics=wanted))
    first_ns = None
    odom_pose = None
    skipped_clouds = 0
    while reader.has_next():
        topic, serialized, timestamp_ns = reader.read_next()
        message = deserialize_message(serialized, message_types[topic])
        if first_ns is None:
            first_ns = timestamp_ns
        t = (timestamp_ns - first_ns) * 1e-9
        if topic == '/cmd_vel_joy':
            samples.commands.append(Command(t, message.linear.x, message.angular.z))
        elif topic == '/odom':
            odom_pose = message.pose.pose
            position = odom_pose.position
            samples.poses.append(Pose(t, position.x, position.y,
                                      _yaw(odom_pose.orientation)))
        elif topic == '/imu':
            # Oak's driver publishes go2_interfaces/IMU, not sensor_msgs/Imu.
            samples.gyro.append(Gyro(t, float(message.gyroscope[2])))
        elif topic == '/scan':
            samples.scans.append(Scan(t, message.angle_min, message.angle_increment,
                                      tuple(message.ranges)))
        elif topic == '/point_cloud2':
            points = _cloud_points(message)
            frame = message.header.frame_id.split('/')[-1]
            if frame == 'odom' and odom_pose is not None:
                p, q = odom_pose.position, odom_pose.orientation
                points = to_base_link(points, (p.x, p.y, p.z), (q.x, q.y, q.z, q.w))
            elif frame != 'base_link':
                skipped_clouds += 1
                continue
            samples.clouds.append(Cloud(t, points))
    if skipped_clouds:
        samples.warnings.append(
            f'{skipped_clouds} clouds skipped: not in base_link, or in odom before '
            'the first /odom message')
    return samples


def build_report(kind, samples):
    """Return the Markdown report and whether anything was measured."""
    if kind == 'object':
        windows = object_windows(samples.scans, samples.clouds)
        return object_report(windows), bool(windows)
    repeats = measure_repeats(samples.commands, samples.poses, samples.gyro)
    report = stop_report(repeats) if kind == 'stop' else turn_report(repeats)
    return report, bool(repeats)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', help='rosbag2 folder recorded in the lab')
    parser.add_argument('--kind', required=True, choices=sorted(KIND_TOPICS))
    args = parser.parse_args(argv)
    samples = read_lab_bag(args.bag, KIND_TOPICS[args.kind])
    report, measured = build_report(args.kind, samples)
    print(report)
    print(f'\nRecording: `{os.path.basename(os.path.normpath(args.bag))}`.')
    for warning in samples.warnings:
        print(f'- Warning: {warning}.')
    return 0 if measured else 1


if __name__ == '__main__':
    raise SystemExit(main())
