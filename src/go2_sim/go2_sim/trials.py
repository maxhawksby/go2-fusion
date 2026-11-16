"""Run the auto mapper N times in the simulator and score each run.

    ros2 run go2_sim trials                       # 10 headless runs
    ros2 run go2_sim trials --runs 3 --timeout 600 -- lidar_rate:=2.0
    ros2 run go2_sim trials --runs 3 --variant blocked_grace \
        --explorer-param blocked_timeout:=2.0

Each run starts a fresh simulator and robot stack (sim.launch.py) at a random
but seeded start pose, starts the auto mapper once Nav2 is up, and waits for
"Exploration complete" or the timeout. Then it scores:

- completeness: % of the building's floor the SLAM map has seen, against a
  ground-truth grid built from the world (world_tools);
- time: from the auto mapper starting to "Exploration complete" (or timeout);
- collisions: separate bumps of Oak's bumper against anything;
- distance: how far Oak really walked.

Results (CSV, a markdown table, each run's final map and logs) go to
src/go2_sim/results/<date-time>/, which git ignores. The table is printed too.
Launch arguments after `--` (e.g. lidar_rate:=2.0 odom_noise_xy:=0.05) go to
every run's sim.launch.py.
"""

import argparse
import csv
import glob
import json
import math
import os
import re
import signal
import subprocess
import sys
import time

import numpy as np

from go2_sim import world_tools

COMPLETE_TEXT = 'Exploration complete'
EXPLORER_NODE = 'frontier_explorer'
# A new bump only after this long with no contact, so one scrape counts once.
BUMP_GAP_SEC = 1.0
# Gazebo can reset or teleport the model while a trial is starting or ending.
# Neither event is physical travel, so keep the independent trial scorer on
# the same discontinuity rule as go2_scoreboard.
MAX_TRUTH_STEP_M = 1.0
# Where the running trial's ROS domain and Gazebo partition are written, so
# `ros2 run go2_sim watch` can attach windows to it. Removed when a run ends.
CURRENT_RUN_FILE = '/tmp/go2_sim_current_run.json'
_PARAMETER_OVERRIDE = re.compile(r'^[A-Za-z_][A-Za-z0-9_.]*:=.+$')
_VARIANT_NAME = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_-]*$')


def explorer_parameter(value):
    """Argparse validator for one ROS parameter override."""

    if not _PARAMETER_OVERRIDE.fullmatch(value):
        raise argparse.ArgumentTypeError(
            'explorer parameters must use name:=value, for example '
            'blocked_timeout:=2.0')
    return value


def variant_name(value):
    """Keep run labels readable and safe to use in artifact names."""

    if not _VARIANT_NAME.fullmatch(value):
        raise argparse.ArgumentTypeError(
            'variant must contain only letters, numbers, underscores or hyphens')
    return value


def explorer_command(parameters):
    """The explorer command with explicit, repeatable ROS parameter dials."""

    command = ['ros2', 'run', 'go2_auto_mapper', 'explorer']
    if parameters:
        command.append('--ros-args')
        for parameter in parameters:
            command.extend(['-p', parameter])
    return command


def default_results_dir():
    here = os.path.realpath(__file__)
    pkg_src = os.path.dirname(os.path.dirname(here))
    if os.path.isfile(os.path.join(pkg_src, 'package.xml')) and os.access(pkg_src, os.W_OK):
        return os.path.join(pkg_src, 'results')
    return os.path.expanduser('~/.ros/go2_sim_results')


def resolve_world(world):
    from ament_index_python.packages import get_package_share_directory
    if world.endswith('.sdf') or os.sep in world:
        path = os.path.abspath(os.path.expanduser(world))
    else:
        path = os.path.join(get_package_share_directory('go2_sim'), 'worlds', world + '.sdf')
    if not os.path.isfile(path):
        sys.exit(f'World not found: {path}')
    return path


def distance_increment(previous, current, max_step=MAX_TRUTH_STEP_M):
    """Return physical travel between poses, excluding reset/teleport jumps."""
    if previous is None:
        return 0.0
    step = math.hypot(current[0] - previous[0], current[1] - previous[1])
    return step if step <= max_step else 0.0


def distance_update(total, previous, current, active):
    """Observe a pose, accumulating it only inside the measurement window."""

    if active:
        total += distance_increment(previous, current)
    return total, current


# ---------------------------------------------------------------------------
# ROS side: one monitor node per run, in that run's ROS domain
# ---------------------------------------------------------------------------

def make_monitor(context):
    from go2_interfaces.msg import AutoMapperEvent
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rcl_interfaces.msg import Log
    from rclpy.node import Node
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                           ReliabilityPolicy, qos_profile_sensor_data)
    from rosgraph_msgs.msg import Clock
    from ros_gz_interfaces.msg import Contacts
    from sensor_msgs.msg import PointCloud2

    class Monitor(Node):
        def __init__(self):
            super().__init__('go2_sim_trials', context=context)
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE,
                                 history=HistoryPolicy.KEEP_LAST)
            self.done = False
            self.measurement_started = False
            self.measurement_started_at = None
            self.latest_map = None
            self.bumps = 0
            self.last_contact = None
            self.distance = 0.0
            self.prev_xy = None
            self.odom_distance = 0.0
            self.prev_odom_xy = None
            self.odom_xy = None
            self.true_xy = None
            self.cloud_seen = False
            self.clock_first = None
            self.clock_last = None
            self.create_subscription(Log, '/rosout', self.on_log, 100)
            self.create_subscription(
                AutoMapperEvent, '/auto_mapper/events', self.on_event, latched)
            self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
            self.create_subscription(Contacts, '/sim/bumper', self.on_contacts, 50)
            self.create_subscription(Odometry, '/sim/ground_truth/odom', self.on_truth, 50)
            self.create_subscription(Odometry, '/odom', self.on_odom, 100)
            self.create_subscription(PointCloud2, '/point_cloud2', self.on_cloud,
                                     qos_profile_sensor_data)
            self.create_subscription(Clock, '/clock', self.on_clock, 10)

        def on_log(self, msg):
            if msg.name == EXPLORER_NODE and COMPLETE_TEXT in msg.msg:
                self.done = True

        def on_event(self, msg):
            if msg.event_type != 'exploration_started' or self.measurement_started:
                return
            # Match the scoreboard's start boundary exactly. The explorer can
            # spend several seconds initializing before this event; counting
            # noisy odometry during that period made parity depend on luck.
            self.measurement_started = True
            self.measurement_started_at = time.monotonic()
            self.distance = 0.0
            self.prev_xy = self.true_xy
            self.odom_distance = 0.0
            self.prev_odom_xy = self.odom_xy
            self.bumps = 0
            self.last_contact = None

        def on_map(self, msg):
            self.latest_map = msg

        def on_contacts(self, msg):
            if not self.measurement_started or not msg.contacts:
                return
            now = time.monotonic()
            if self.last_contact is None or now - self.last_contact > BUMP_GAP_SEC:
                self.bumps += 1
            self.last_contact = now

        def on_truth(self, msg):
            p = msg.pose.pose.position
            current = (p.x, p.y)
            self.distance, self.prev_xy = distance_update(
                self.distance, self.prev_xy, current, self.measurement_started)
            self.true_xy = current

        def on_odom(self, msg):
            p = msg.pose.pose.position
            current = (p.x, p.y)
            self.odom_distance, self.prev_odom_xy = distance_update(
                self.odom_distance, self.prev_odom_xy, current,
                self.measurement_started)
            self.odom_xy = current

        def on_cloud(self, _msg):
            self.cloud_seen = True

        def on_clock(self, msg):
            sim = msg.clock.sec + msg.clock.nanosec * 1e-9
            sample = (time.monotonic(), sim)
            if self.clock_first is None:
                self.clock_first = sample
            self.clock_last = sample

        def real_time_factor(self):
            if self.clock_first is None or self.clock_last is None:
                return float('nan')
            wall = self.clock_last[0] - self.clock_first[0]
            return (self.clock_last[1] - self.clock_first[1]) / wall if wall > 1 else float('nan')

    return Monitor()


def slam_grid(msg):
    info = msg.info
    data = np.array(msg.data, dtype=np.int16).reshape(info.height, info.width)
    return world_tools.Grid(data, info.resolution,
                            info.origin.position.x, info.origin.position.y)


def save_slam_map(grid, path):
    out = np.full(grid.data.shape, 255, dtype=np.uint8)
    out[(grid.data >= 0) & (grid.data < 25)] = world_tools.FREE
    out[grid.data >= 65] = world_tools.OCCUPIED
    world_tools.write_map(world_tools.Grid(out, grid.resolution, grid.origin_x,
                                           grid.origin_y), path)


# ---------------------------------------------------------------------------
# Processes
# ---------------------------------------------------------------------------

def stop_group(proc, first_signal=signal.SIGINT, wait=20.0):
    """Stop a process and everything it started (its own process group)."""
    if proc is None or proc.poll() is not None:
        return
    for sig, timeout in ((first_signal, wait), (signal.SIGTERM, 5.0), (signal.SIGKILL, 5.0)):
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            return
        try:
            proc.wait(timeout=timeout)
            return
        except subprocess.TimeoutExpired:
            continue


def finish_scoreboard(proc, env, wait=15.0):
    """Ask the recorder to finalize before falling back to process signals."""
    if proc is None or proc.poll() is not None:
        return
    try:
        subprocess.run(
            ['ros2', 'service', 'call', '/scoreboard/finish',
             'std_srvs/srv/Trigger', '{}'],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            env=env,
            timeout=10.0,
            check=False,
        )
        proc.wait(timeout=wait)
    except (OSError, subprocess.TimeoutExpired):
        stop_group(proc, wait=wait)


def write_current_run(index, runs, env):
    info = {k: env.get(k, '') for k in
            ('ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY', 'IGN_PARTITION')}
    info.update(run=index + 1, runs=runs)
    try:
        with open(CURRENT_RUN_FILE + '.tmp', 'w') as f:
            json.dump(info, f)
        os.replace(CURRENT_RUN_FILE + '.tmp', CURRENT_RUN_FILE)
    except OSError:
        pass


def clear_current_run():
    try:
        os.remove(CURRENT_RUN_FILE)
    except OSError:
        pass


def scoreboard_parity(legacy, scoreboard):
    """Compare the independent scorers with explicit, reviewable tolerances."""
    try:
        # The scoreboard measures Oak's reported /odom, not Gazebo's hidden
        # ground-truth pose. Keep physical travel as a separate benchmark and
        # compare two independent accumulators fed by the same public signal.
        legacy_distance = float(legacy['odom_distance_m'])
        score_distance = float(scoreboard['distance_m'])
        legacy_completeness = float(legacy['completeness_pct'])
        score_completeness = float(scoreboard['completeness_pct'])
        legacy_collisions = int(legacy['collisions'])
        score_collisions = int(scoreboard['collisions'])
    except (KeyError, TypeError, ValueError):
        checks = {'distance': False, 'completeness': False, 'collisions': False}
        return checks, ['scoreboard metrics unavailable']
    checks = {
        'distance': abs(legacy_distance - score_distance)
        <= max(0.5, 0.10 * max(legacy_distance, 0.1)),
        'completeness': abs(legacy_completeness - score_completeness) <= 1.0,
        'collisions': legacy_collisions == score_collisions,
    }
    return checks, [name for name, passed in checks.items() if not passed]


def _latest_scoreboard_summary(root):
    paths = glob.glob(os.path.join(root, '*', 'run.json'))
    if not paths:
        return None
    path = max(paths, key=os.path.getmtime)
    with open(path, encoding='utf-8') as stream:
        return json.load(stream)


def run_one(args, index, world_path, gt, gt_path, pose, run_seed, out_dir, launch_extra):
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
    from rclpy.executors import SingleThreadedExecutor
    from std_srvs.srv import Trigger

    domain = 50 + (index % 40)   # a fresh ROS domain per run: no leftovers can talk to it
    env = os.environ.copy()
    env.update(ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY='1',
               IGN_PARTITION=f'go2_sim_trials_{os.getpid()}_{index}')
    logs = os.path.join(out_dir, 'logs')
    os.makedirs(logs, exist_ok=True)
    tag = f'run_{index + 1:02d}'
    x, y, yaw = pose
    launch_cmd = ['ros2', 'launch', 'go2_sim', 'sim.launch.py',
                  f'world:={world_path}', f'x:={x:.3f}', f'y:={y:.3f}', f'yaw:={yaw:.4f}',
                  'gui:=false', 'rviz:=false', 'explore:=false',
                  f'domain_id:={domain}', f'seed:={run_seed}'] + launch_extra
    result = dict(run=index + 1, seed=run_seed, start_x=round(x, 2), start_y=round(y, 2),
                  start_yaw=round(yaw, 2), completeness_pct=0.0, time_s=0.0, collisions=0,
                  distance_m=0.0, finished='no', end_reason='', rtf=float('nan'),
                  end_x=float('nan'), end_y=float('nan'),
                  odom_distance_m=0.0, odom_error_m=0.0, odom_error_pct='',
                  scoreboard_distance_m='', scoreboard_completeness_pct='',
                  scoreboard_collisions='', scoreboard_parity='disabled')

    launch_log = open(os.path.join(logs, f'{tag}_launch.log'), 'w')
    explorer_log = open(os.path.join(logs, f'{tag}_explorer.log'), 'w')
    scoreboard_log = open(os.path.join(logs, f'{tag}_scoreboard.log'), 'w')
    launch = subprocess.Popen(launch_cmd, stdout=launch_log, stderr=subprocess.STDOUT,
                              env=env, start_new_session=True)
    write_current_run(index, args.runs, env)
    explorer = None
    scoreboard = None
    scoreboard_root = os.path.join(out_dir, 'scoreboard', tag)
    context = rclpy.Context()
    # Our own Ctrl+C / SIGTERM handling (main), so the finally below always
    # stops this run's simulator and auto mapper.
    rclpy.init(context=context, domain_id=domain,
               signal_handler_options=SignalHandlerOptions.NO)
    executor = SingleThreadedExecutor(context=context)
    node = make_monitor(context)
    executor.add_node(node)
    started = None
    try:
        # 1. Wait for the stack: lidar flowing and Nav2 accepting goals.
        nav = ActionClient(node, NavigateToPose, '/navigate_to_pose')
        deadline = time.monotonic() + args.startup_timeout
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.2)
            if launch.poll() is not None:
                result['end_reason'] = f'sim exited during startup (code {launch.returncode})'
                return result
            if node.cloud_seen and nav.server_is_ready():
                break
        else:
            result['end_reason'] = ('startup timeout: ' + ('Nav2 not ready' if node.cloud_seen
                                                           else 'no lidar on /point_cloud2'))
            return result

        # 2. Start the measurement-only scoreboard before the mapper so its
        # first exploration event cannot be missed.
        if not args.no_scoreboard:
            scoreboard_command = [
                'ros2', 'run', 'go2_scoreboard', 'recorder', '--ros-args',
                '-p', 'use_sim_time:=true',
                '-p', 'source:=sim',
                '-p', f'run_label:={args.variant}_seed_{run_seed}',
                '-p', f'results_root:={scoreboard_root}',
                '-p', f'record_bag:={str(not args.no_scoreboard_bag).lower()}',
                '-p', f'ground_truth_map:={gt_path}',
                '-p', f'start_x:={x:.6f}', '-p', f'start_y:={y:.6f}',
                '-p', f'start_yaw:={yaw:.6f}',
                '-p', "configuration_json:='"
                + json.dumps({
                    'algorithm': 'frontier',
                    'variant': args.variant,
                    'seed': run_seed,
                    'launch_args': launch_extra,
                    'explorer_params': args.explorer_param,
                }, separators=(',', ':')) + "'",
            ]
            scoreboard = subprocess.Popen(
                scoreboard_command, stdout=scoreboard_log, stderr=subprocess.STDOUT,
                env=env, start_new_session=True)
            # The recorder computes its source fingerprint and then starts a
            # separate rosbag process. A fixed sleep raced both on slower
            # machines and produced bags missing the start event and initial
            # odometry. Wait for the actual ROS graph instead.
            ready_client = node.create_client(Trigger, '/scoreboard/ready')
            scoreboard_deadline = time.monotonic() + min(30.0, args.startup_timeout)
            while time.monotonic() < scoreboard_deadline:
                executor.spin_once(timeout_sec=0.2)
                if scoreboard.poll() is not None:
                    result['end_reason'] = (
                        f'scoreboard exited during startup (code {scoreboard.returncode})')
                    return result
                if ready_client.service_is_ready():
                    break
            else:
                result['end_reason'] = 'scoreboard startup timeout'
                return result
            node.destroy_client(ready_client)

        # 3. Start the auto mapper, exactly as the RViz panel does.
        explorer = subprocess.Popen(explorer_command(args.explorer_param),
                                    stdout=explorer_log, stderr=subprocess.STDOUT,
                                    env=env, start_new_session=True)
        # The scoreboard and independent scorer both begin at the structured
        # start event, not at process creation. Wait for that shared boundary.
        start_deadline = time.monotonic() + min(30.0, args.startup_timeout)
        while time.monotonic() < start_deadline and not node.measurement_started:
            executor.spin_once(timeout_sec=0.2)
            if explorer.poll() is not None:
                result['end_reason'] = (
                    f'auto mapper exited before start event (code {explorer.returncode})')
                return result
        if not node.measurement_started:
            result['end_reason'] = 'auto mapper start event timeout'
            return result
        started = node.measurement_started_at
        deadline = started + args.timeout
        anchor, anchor_time = node.true_xy, started   # for spotting a stuck robot
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.2)
            now = time.monotonic()
            if node.true_xy is not None and (anchor is None or math.hypot(
                    node.true_xy[0] - anchor[0], node.true_xy[1] - anchor[1]) > 0.25):
                anchor, anchor_time = node.true_xy, now
            if args.stall_timeout > 0 and now - anchor_time > args.stall_timeout:
                result['end_reason'] = f'stuck: moved < 0.25 m in {args.stall_timeout:.0f} s'
                break
            if node.done:
                result['finished'] = 'yes'
                result['end_reason'] = 'exploration complete'
                break
            if launch.poll() is not None:
                result['end_reason'] = f'sim exited (code {launch.returncode})'
                break
            if explorer.poll() is not None:
                result['end_reason'] = f'auto mapper exited (code {explorer.returncode})'
                break
        else:
            result['end_reason'] = 'timeout'
        result['time_s'] = round(time.monotonic() - started, 1)

        # Freeze exploration at the measurement boundary. Previously the
        # mapper kept moving while the legacy scorer took its map snapshot and
        # the scoreboard finalized, so the two independent scorers measured
        # different moments of the same run.
        stop_group(explorer, wait=8.0)

        # Let the clean explorer shutdown and SLAM's final update propagate.
        settle = time.monotonic() + 3.0
        while time.monotonic() < settle:
            executor.spin_once(timeout_sec=0.2)

        result['collisions'] = node.bumps
        result['distance_m'] = round(node.distance, 1)
        result['odom_distance_m'] = round(node.odom_distance, 4)
        result['odom_error_m'] = round(node.odom_distance - node.distance, 4)
        if node.distance > 1e-6:
            result['odom_error_pct'] = round(
                100.0 * (node.odom_distance - node.distance) / node.distance, 2)
        result['rtf'] = round(node.real_time_factor(), 2)
        if node.true_xy is not None:
            result['end_x'], result['end_y'] = (round(v, 2) for v in node.true_xy)
        if node.latest_map is not None:
            grid = slam_grid(node.latest_map)
            result['completeness_pct'] = round(
                world_tools.score_completeness(gt, grid, pose), 1)
            save_slam_map(grid, os.path.join(out_dir, 'maps', f'{tag}_map.yaml'))
        if scoreboard is not None:
            finish_scoreboard(scoreboard, env)
            scoreboard_summary = _latest_scoreboard_summary(scoreboard_root)
            if scoreboard_summary is None:
                result['scoreboard_parity'] = 'missing scoreboard report'
            else:
                result['scoreboard_distance_m'] = scoreboard_summary['distance_m']
                result['scoreboard_completeness_pct'] = scoreboard_summary['completeness_pct']
                result['scoreboard_collisions'] = scoreboard_summary['collisions']
                _checks, failures = scoreboard_parity(result, scoreboard_summary)
                result['scoreboard_parity'] = (
                    'pass' if not failures else 'fail: ' + ', '.join(failures))
        return result
    finally:
        clear_current_run()
        stop_group(scoreboard, wait=15.0)
        stop_group(explorer, wait=8.0)
        stop_group(launch, wait=20.0)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)
        launch_log.close()
        explorer_log.close()
        scoreboard_log.close()


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------

COLUMNS = ['run', 'seed', 'start_x', 'start_y', 'start_yaw', 'completeness_pct', 'time_s',
           'collisions', 'distance_m', 'odom_distance_m', 'odom_error_m', 'odom_error_pct',
           'finished', 'end_reason', 'end_x', 'end_y', 'rtf']
COLUMNS += ['scoreboard_distance_m', 'scoreboard_completeness_pct',
            'scoreboard_collisions', 'scoreboard_parity']


def table(results):
    head = ('| Run | Start (x, y, yaw) | Map complete | Time | Collisions | Distance '
            '| Finished | Note |')
    lines = [head, '|---:|---|---:|---:|---:|---:|---|---|']
    for r in results:
        lines.append(
            f"| {r['run']} | {r['start_x']:.2f}, {r['start_y']:.2f}, {r['start_yaw']:.2f} "
            f"| {r['completeness_pct']:.1f} % | {r['time_s']:.0f} s | {r['collisions']} "
            f"| {r['distance_m']:.1f} m | {r['finished']} | {r['end_reason']}"
            + (f" (ended at {r['end_x']:.1f}, {r['end_y']:.1f})" if r['finished'] != 'yes'
               and r['end_x'] == r['end_x'] else '') + ' |')
    if results:
        n = len(results)
        done = sum(r['finished'] == 'yes' for r in results)
        mean_c = sum(r['completeness_pct'] for r in results) / n
        min_c = min(r['completeness_pct'] for r in results)
        total_t = sum(r['time_s'] for r in results)
        lines.append(
            f"| **All {n}** | | **mean {mean_c:.1f} %** (min {min_c:.1f} %) "
            f"| **{total_t:.0f} s** (mean {total_t / n:.0f} s) "
            f"| **{sum(r['collisions'] for r in results)}** "
            f"| **{sum(r['distance_m'] for r in results):.1f} m** "
            f"| **{done}/{n}** | |")
    return '\n'.join(lines)


def write_results(out_dir, results, header):
    with open(os.path.join(out_dir, 'results.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=COLUMNS)
        w.writeheader()
        for r in results:
            w.writerow({k: r[k] for k in COLUMNS})
    with open(os.path.join(out_dir, 'results.md'), 'w') as f:
        f.write(header + '\n\n' + table(results) + '\n')


def _interrupt(_signum, _frame):
    raise KeyboardInterrupt


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    launch_extra = []
    if '--' in argv:
        i = argv.index('--')
        argv, launch_extra = argv[:i], argv[i + 1:]
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--runs', type=int, default=10)
    parser.add_argument('--seed', type=int, default=1, help='same seed = same start poses')
    parser.add_argument('--timeout', type=float, default=900.0,
                        help='seconds the auto mapper gets per run')
    parser.add_argument('--stall-timeout', type=float, default=180.0,
                        help='end a run when Oak moves less than 0.25 m in this many '
                        'seconds (0 = never)')
    parser.add_argument('--startup-timeout', type=float, default=180.0,
                        help='seconds for the simulator and Nav2 to come up')
    parser.add_argument('--world', default='placeholder_rooms')
    parser.add_argument('--clearance', type=float, default=0.7,
                        help='free space (m) around each random start pose')
    parser.add_argument('--out', default=None, help='results folder')
    parser.add_argument('--no-scoreboard', action='store_true',
                        help='run only the legacy simulator scorer')
    parser.add_argument('--no-scoreboard-bag', action='store_true',
                        help='score runs but skip rosbag recording')
    parser.add_argument('--variant', type=variant_name, default='frontier',
                        help='short experiment label stored with every run')
    parser.add_argument('--explorer-param', action='append', default=[],
                        type=explorer_parameter, metavar='NAME:=VALUE',
                        help='temporary go2_auto_mapper ROS parameter; repeat as needed')
    args = parser.parse_args(argv)

    # Stopping the runner (Ctrl+C, SIGTERM, docker stop) unwinds through each
    # run's cleanup instead of leaving its simulator running.
    signal.signal(signal.SIGTERM, _interrupt)
    signal.signal(signal.SIGINT, signal.default_int_handler)
    # The monitor must not reach a real robot's network either.
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    world_path = resolve_world(args.world)
    out_dir = os.path.join(args.out or default_results_dir(), time.strftime('%Y%m%d-%H%M%S'))
    os.makedirs(os.path.join(out_dir, 'maps'), exist_ok=True)
    gt, gt_path, notes = world_tools.ground_truth_for_world(world_path, out_dir)
    for note in notes:
        print(f'[trials] warning: {note}')
    header = (f'# Auto mapper trials\n\nWorld: `{os.path.basename(world_path)}`, '
              f'{args.runs} runs, seed {args.seed}, timeout {args.timeout:.0f} s, '
              f'variant: `{args.variant}`, explorer parameters: '
              f'`{" ".join(args.explorer_param) or "defaults"}`, '
              f'launch args: `{" ".join(launch_extra) or "defaults"}`. '
              f'Ground truth: `{os.path.basename(gt_path)}`.')
    print(f'[trials] {args.runs} runs in {os.path.basename(world_path)}; results in {out_dir}',
          flush=True)

    results = []
    try:
        run_all(args, world_path, gt, gt_path, out_dir, header, launch_extra, results)
    except KeyboardInterrupt:
        print('[trials] stopped early; each started run was shut down.', flush=True)

    print('\n' + table(results) + '\n', flush=True)
    print(f'[trials] wrote {out_dir}/results.csv and results.md', flush=True)


def run_all(args, world_path, gt, gt_path, out_dir, header, launch_extra, results):
    startup_failures = 0
    for i in range(args.runs):
        run_seed = args.seed * 1000 + i
        pose = world_tools.sample_start_pose(gt, np.random.default_rng(run_seed),
                                             clearance=args.clearance)
        print(f'[trials] run {i + 1}/{args.runs}: start {pose[0]:.2f}, {pose[1]:.2f}, '
              f'yaw {pose[2]:.2f}', flush=True)
        r = run_one(args, i, world_path, gt, gt_path, pose, run_seed, out_dir, launch_extra)
        results.append(r)
        write_results(out_dir, results, header)
        print(f"[trials] run {i + 1}: {r['completeness_pct']:.1f} % in {r['time_s']:.0f} s, "
              f"{r['collisions']} collisions, finished {r['finished']} ({r['end_reason']}), "
              f"real-time factor {r['rtf']}", flush=True)
        startup_failures = startup_failures + 1 if 'startup' in r['end_reason'] else 0
        if startup_failures >= 2:
            print('[trials] the simulator failed to start twice in a row; stopping. '
                  f'See {out_dir}/logs.', flush=True)
            break


if __name__ == '__main__':
    main()
