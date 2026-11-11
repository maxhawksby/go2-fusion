"""Regenerate a scorecard directly from a rosbag2 recording."""

import argparse
from datetime import datetime, timezone
import os

from go2_scoreboard.accumulator import RECORDING, RunAccumulator
from go2_scoreboard.artifacts import write_artifacts
from go2_scoreboard.map_metrics import CompletenessScorer, grid_from_values, read_map


def _stamp_seconds(message, bag_timestamp_ns):
    header = getattr(message, 'header', None)
    stamp = getattr(message, 'stamp', None) or getattr(header, 'stamp', None)
    if stamp is not None:
        seconds = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        if seconds > 0.0:
            return seconds
    return float(bag_timestamp_ns) * 1e-9


def replay_bag(bag_path, output_dir, ground_truth_path='', start_pose=(0.0, 0.0, 0.0)):
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as error:
        raise RuntimeError('rosbag2_py and ROS message support are required for replay') from error

    storage = rosbag2_py.StorageOptions(
        uri=os.path.abspath(bag_path), storage_id='sqlite3')
    converter = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage, converter)
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    message_types = {}
    warnings = []
    for topic, type_name in topic_types.items():
        try:
            message_types[topic] = get_message(type_name)
        except (AttributeError, ModuleNotFoundError, ValueError) as error:
            warnings.append(f'could not load {type_name} on {topic}: {error}')

    # Bags deliberately begin while the recorder is WAITING, so they contain
    # pre-mission odometry and maps. Discover whether a real start event exists
    # before accumulating anything; this keeps memory bounded for large bags.
    has_start_event = False
    event_type = message_types.get('/auto_mapper/events')
    if event_type is not None:
        while reader.has_next():
            topic, serialized, _timestamp_ns = reader.read_next()
            if topic != '/auto_mapper/events':
                continue
            event = deserialize_message(serialized, event_type)
            if event.event_type == 'exploration_started':
                has_start_event = True
                break
    reader = rosbag2_py.SequentialReader()
    reader.open(storage, converter)

    run_id = datetime.now(timezone.utc).strftime('%Y%m%d-%H%M%S_replay')
    accumulator = RunAccumulator(run_id=run_id, source='replay', run_label='replay')
    accumulator.configuration = {
        'source_bag': os.path.abspath(bag_path),
        'ground_truth_map': os.path.abspath(ground_truth_path) if ground_truth_path else None,
        'start_pose': list(start_pose),
    }
    accumulator.warnings.extend(warnings)
    ground_truth = read_map(ground_truth_path) if ground_truth_path else None
    scorer = CompletenessScorer(ground_truth, start_pose) if ground_truth is not None else None
    last_contact_timestamp = None
    while reader.has_next():
        topic, serialized, timestamp_ns = reader.read_next()
        message_type = message_types.get(topic)
        if message_type is None:
            continue
        message = deserialize_message(serialized, message_type)
        timestamp = _stamp_seconds(message, timestamp_ns)
        if topic == '/auto_mapper/events':
            accumulator.handle_event(message.event_type, message.goal_sequence,
                                     timestamp, message.detail)
        elif topic == '/odom':
            if accumulator.state != RECORDING:
                if has_start_event:
                    continue
                accumulator.start(timestamp)
                accumulator._warn_once('replay had no exploration_started event before odometry')
            position = message.pose.pose.position
            accumulator.update_odom(position.x, position.y, timestamp)
        elif topic == '/map':
            if accumulator.state != RECORDING:
                if has_start_event:
                    continue
                accumulator.start(timestamp)
                accumulator._warn_once('replay had no exploration_started event before map data')
            info = message.info
            grid = grid_from_values(message.data, info.width, info.height, info.resolution,
                                    info.origin.position.x, info.origin.position.y)
            completeness = scorer.score(grid) if scorer is not None else None
            accumulator.update_map(grid, timestamp, completeness)
        elif topic == '/scoreboard/status':
            if message.run_id:
                accumulator.run_id = message.run_id
            if accumulator.state == RECORDING:
                accumulator.add_resource_sample(timestamp, message.cpu_pct, message.memory_mb)
                # The live recorder publishes FINALIZING before it stops the
                # bag process. Ignore sensor messages captured during bag
                # flush so replay uses the same mission window as live scoring.
                if message.state in ('FINALIZING', 'COMPLETE', 'ERROR'):
                    break
        elif topic == '/sim/bumper' and accumulator.state == RECORDING:
            if getattr(message, 'contacts', None):
                if (last_contact_timestamp is None
                        or timestamp - last_contact_timestamp > 1.0):
                    accumulator.mark_collision()
                last_contact_timestamp = timestamp

    if accumulator.state != RECORDING:
        accumulator.start(0.0)
        accumulator._warn_once('bag did not contain mapping events, odometry or map data')
    if accumulator.latest_time_s is None:
        accumulator.latest_time_s = accumulator.start_time_s
    accumulator.sample_timeline()
    accumulator.begin_finalizing()
    accumulator.finalize(accumulator.end_reason or 'bag replay complete')
    replay_dir = os.path.join(os.path.abspath(output_dir), f'{accumulator.run_id}_replay')
    os.makedirs(replay_dir, exist_ok=False)
    return write_artifacts(replay_dir, accumulator), replay_dir


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', help='rosbag2 directory to replay')
    parser.add_argument('--out', default=os.path.expanduser('~/.ros/go2_scoreboard_replays'))
    parser.add_argument('--ground-truth', default='')
    parser.add_argument('--start-x', type=float, default=0.0)
    parser.add_argument('--start-y', type=float, default=0.0)
    parser.add_argument('--start-yaw', type=float, default=0.0)
    args = parser.parse_args(argv)
    summary, output = replay_bag(
        args.bag, args.out, args.ground_truth,
        (args.start_x, args.start_y, args.start_yaw))
    print(f"replayed {args.bag}: {summary['distance_m']:.2f} m, "
          f"{summary['known_area_m2']:.2f} m² known; wrote {output}")


if __name__ == '__main__':
    main()
