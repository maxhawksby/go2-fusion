"""Live ROS 2 run recorder for Oak and the simulator."""

from datetime import datetime, timezone
import json
import math
import os
import re
import time

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

from go2_interfaces.msg import AutoMapperEvent, ScoreboardStatus
from go2_scoreboard.accumulator import RECORDING, RunAccumulator
from go2_scoreboard.artifacts import write_artifacts
from go2_scoreboard.bag_recorder import BagRecorder
from go2_scoreboard.map_metrics import grid_from_values, read_map, score_completeness
from go2_scoreboard.resources import ResourceSampler
from go2_scoreboard.versioning import source_fingerprint


def default_results_dir():
    here = os.path.realpath(__file__)
    package_source = os.path.dirname(os.path.dirname(here))
    if (os.path.isfile(os.path.join(package_source, 'package.xml'))
            and os.access(package_source, os.W_OK)):
        return os.path.join(package_source, 'results')
    return os.path.expanduser('~/.ros/go2_scoreboard_results')


def _safe_label(label):
    cleaned = re.sub(r'[^A-Za-z0-9_.-]+', '-', str(label).strip()).strip('-')
    return cleaned[:80] or 'run'


def _stamp_seconds(stamp, fallback):
    seconds = float(stamp.sec) + float(stamp.nanosec) * 1e-9
    return seconds if seconds > 0.0 else float(fallback)


class ScoreboardRecorder(Node):
    def __init__(self):
        super().__init__('go2_scoreboard')
        self.declare_parameter('source', 'oak')
        self.declare_parameter('run_label', 'default')
        self.declare_parameter('results_root', default_results_dir())
        self.declare_parameter('record_bag', True)
        self.declare_parameter('ground_truth_map', '')
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_yaw', 0.0)
        self.declare_parameter('max_odom_step_m', 1.0)
        self.declare_parameter('code_version', os.environ.get('GO2_CODE_VERSION', 'unknown'))
        self.declare_parameter('source_root', '/ros2_ws/src')
        self.declare_parameter('configuration_json', '{}')

        self.source = str(self.get_parameter('source').value)
        self.run_label = str(self.get_parameter('run_label').value)
        timestamp = datetime.now(timezone.utc).strftime('%Y%m%d-%H%M%S')
        self.run_id = f'{timestamp}_{_safe_label(self.run_label)}'
        results_root = os.path.abspath(os.path.expanduser(
            str(self.get_parameter('results_root').value)))
        self.run_dir = os.path.join(results_root, self.run_id)
        os.makedirs(self.run_dir, exist_ok=False)
        source_root = str(self.get_parameter('source_root').value)
        self.accumulator = RunAccumulator(
            run_id=self.run_id,
            source=self.source,
            run_label=self.run_label,
            code_version=str(self.get_parameter('code_version').value),
            source_hash=source_fingerprint(source_root),
            max_odom_step_m=float(self.get_parameter('max_odom_step_m').value),
        )
        self.ground_truth = None
        self.start_pose = (
            float(self.get_parameter('start_x').value),
            float(self.get_parameter('start_y').value),
            float(self.get_parameter('start_yaw').value),
        )
        ground_truth_path = str(self.get_parameter('ground_truth_map').value).strip()
        self.accumulator.configuration = {
            'source': self.source,
            'run_label': self.run_label,
            'start_pose': list(self.start_pose),
            'ground_truth_map': ground_truth_path or None,
            'record_bag': bool(self.get_parameter('record_bag').value),
            'max_odom_step_m': float(self.get_parameter('max_odom_step_m').value),
        }
        try:
            experiment = json.loads(
                str(self.get_parameter('configuration_json').value))
            if not isinstance(experiment, dict):
                raise ValueError('configuration_json must contain a JSON object')
            self.accumulator.configuration['parameters'] = experiment
        except (TypeError, ValueError, json.JSONDecodeError) as error:
            self.accumulator.configuration['parameters'] = {}
            self.accumulator.warnings.append(f'invalid configuration_json: {error}')
        if ground_truth_path:
            try:
                self.ground_truth = read_map(os.path.expanduser(ground_truth_path))
            except (OSError, ValueError, KeyError) as error:
                self.accumulator.warnings.append(f'could not load ground truth: {error}')

        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.status_publisher = self.create_publisher(
            ScoreboardStatus, '/scoreboard/status', latched)
        event_qos = QoSProfile(
            depth=100,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(AutoMapperEvent, '/auto_mapper/events',
                                 self._on_event, event_qos)
        self.create_subscription(Odometry, '/odom', self._on_odom, 100)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, latched)
        self.create_service(Trigger, '/scoreboard/mark_human_touch',
                            self._mark_human_touch)
        self.create_service(Trigger, '/scoreboard/mark_collision', self._mark_collision)
        self.create_service(Trigger, '/scoreboard/finish', self._finish_service)
        self._create_optional_bumper_subscription()
        self.resource_sampler = ResourceSampler()
        self.last_resource_sample = time.monotonic()
        self.last_contact = None
        self.bag_recorder = None
        if bool(self.get_parameter('record_bag').value):
            self.bag_recorder = BagRecorder(os.path.join(self.run_dir, 'bag'))
            try:
                self.bag_recorder.start()
                if not self.bag_recorder.wait_ready():
                    self.accumulator.warnings.append(
                        'bag recorder did not become ready before the run')
            except OSError as error:
                self.accumulator.warnings.append(f'bag recorder did not start: {error}')
                self.bag_recorder = None
        self.ready_service = None
        if self.bag_recorder is None or self.bag_recorder.ready():
            self.ready_service = self.create_service(
                Trigger, '/scoreboard/ready', self._ready)
        self.finished = False
        self.final_summary = None
        self.create_timer(1.0, self._tick)
        self.get_logger().info(
            f'Scoreboard waiting for exploration; artifacts: {self.run_dir}')

    def _create_optional_bumper_subscription(self):
        try:
            from ros_gz_interfaces.msg import Contacts
        except ImportError:
            self.accumulator.warnings.append('simulator bumper message type is unavailable')
            return
        self.create_subscription(Contacts, '/sim/bumper', self._on_contacts, 50)

    def _ready(self, _request, response):
        response.success = True
        response.message = f'scoreboard ready for {self.run_id}'
        return response

    def _now_seconds(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        return now if now > 0 else time.time()

    def _on_event(self, message):
        timestamp = _stamp_seconds(message.stamp, self._now_seconds())
        self.accumulator.handle_event(
            message.event_type, message.goal_sequence, timestamp, message.detail)
        if message.event_type in ('exploration_completed', 'exploration_stopped'):
            self.finalize(message.detail or message.event_type)

    def _on_odom(self, message):
        timestamp = _stamp_seconds(message.header.stamp, self._now_seconds())
        position = message.pose.pose.position
        self.accumulator.update_odom(position.x, position.y, timestamp)

    def _on_map(self, message):
        timestamp = _stamp_seconds(message.header.stamp, self._now_seconds())
        info = message.info
        grid = grid_from_values(message.data, info.width, info.height, info.resolution,
                                info.origin.position.x, info.origin.position.y)
        completeness = None
        if self.ground_truth is not None:
            completeness = score_completeness(
                self.ground_truth, grid, self.start_pose)
        self.accumulator.update_map(grid, timestamp, completeness)

    def _on_contacts(self, message):
        if self.accumulator.state != RECORDING or not message.contacts:
            return
        now = time.monotonic()
        if self.last_contact is None or now - self.last_contact > 1.0:
            self.accumulator.mark_collision()
        self.last_contact = now

    def _mark_human_touch(self, _request, response):
        if self.accumulator.state != RECORDING:
            response.success = False
            response.message = 'no mapping run is recording'
            return response
        self.accumulator.mark_human_touch()
        response.success = True
        response.message = f'human touches: {self.accumulator.human_touches}'
        return response

    def _mark_collision(self, _request, response):
        if self.accumulator.state != RECORDING:
            response.success = False
            response.message = 'no mapping run is recording'
            return response
        self.accumulator.mark_collision()
        response.success = True
        response.message = f'collisions: {self.accumulator.collisions}'
        return response

    def _finish_service(self, _request, response):
        if self.finished:
            response.success = False
            response.message = 'run is already finalized'
            return response
        if self.accumulator.state != RECORDING:
            self.accumulator.start(self._now_seconds())
            self.accumulator.warnings.append(
                'run was manually finalized before exploration started')
        self.finalize('operator requested finish')
        response.success = True
        response.message = f'finalized {self.run_id}'
        return response

    def _tick(self):
        if self.finished:
            return
        now_mono = time.monotonic()
        if self.accumulator.state == RECORDING:
            interval = now_mono - self.last_resource_sample
            cpu, memory, processes = self.resource_sampler.sample(interval)
            self.last_resource_sample = now_mono
            self.accumulator.add_resource_sample(self._now_seconds(), cpu, memory, processes)
            self.accumulator.sample_timeline()
            try:
                write_artifacts(self.run_dir, self.accumulator, append_index=False)
            except OSError as error:
                self.accumulator._warn_once(f'checkpoint write failed: {error}')
        if self.bag_recorder is not None and not self.bag_recorder.healthy():
            self.accumulator._warn_once('bag recorder exited before the run finished')
        self._publish_status()

    def _publish_status(self):
        summary = self.accumulator.summary()
        message = ScoreboardStatus()
        message.stamp = self.get_clock().now().to_msg()
        message.run_id = self.run_id
        message.state = summary['state']
        message.elapsed_s = summary['duration_s']
        message.distance_m = summary['distance_m']
        message.known_area_m2 = summary['known_area_m2']
        message.completeness_pct = (math.nan if summary['completeness_pct'] is None
                                    else summary['completeness_pct'])
        message.goals_sent = summary['goals_sent']
        message.goals_reached = summary['goals_reached']
        message.goals_failed = summary['goals_failed']
        message.goals_cancelled = summary['goals_cancelled']
        message.recoveries = summary['recoveries']
        message.collisions = summary['collisions']
        message.safety_stops = summary['safety_stops']
        message.human_touches = summary['human_touches']
        message.cpu_pct = self.accumulator.cpu_pct
        message.memory_mb = self.accumulator.memory_mb
        message.end_reason = summary['end_reason']
        message.warnings = summary['warnings']
        try:
            self.status_publisher.publish(message)
        except Exception as error:
            # Artifact finalization must still succeed if ROS is already
            # shutting down or the status publisher becomes unavailable.
            self.accumulator._warn_once(f'status publish failed: {error}')

    def finalize(self, reason='run finalized', error=False):
        if self.finished:
            return self.final_summary
        if self.accumulator.state == RECORDING:
            self.accumulator.sample_timeline()
        self.accumulator.begin_finalizing()
        self._publish_status()
        self.accumulator.finalize(reason, error=error)
        self._publish_status()
        if self.bag_recorder is not None:
            self.bag_recorder.stop()
        try:
            self.final_summary = write_artifacts(self.run_dir, self.accumulator)
        except OSError as artifact_error:
            self.accumulator._warn_once(f'final artifact write failed: {artifact_error}')
            self.final_summary = self.accumulator.summary()
        self.finished = True
        self.get_logger().info(
            f"Scoreboard finalized {self.run_id}: {self.final_summary['end_reason']}")
        return self.final_summary


def main(args=None):
    from rclpy.signals import SignalHandlerOptions
    # Python receives KeyboardInterrupt while the context is still valid, so
    # finalize() can publish COMPLETE and atomically write the final snapshot.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = ScoreboardRecorder()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        if node.accumulator.state != RECORDING:
            node.accumulator.start(node._now_seconds())
            node.accumulator.warnings.append('recorder stopped before exploration started')
        node.finalize('recorder interrupted')
    except Exception as error:
        node.finalize(f'recorder error: {error}', error=True)
        raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
