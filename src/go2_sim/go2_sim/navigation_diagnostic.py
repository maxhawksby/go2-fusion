"""Summarize recorded sensing and navigation evidence without replaying motion.

Run with the container's Python after sourcing the ROS workspace:
python3 /ros2_ws/src/go2_sim/go2_sim/navigation_diagnostic.py <results-directory>
"""

import argparse
from collections import Counter, defaultdict
import json
import math
from pathlib import Path


def summarize(records):
    """Consume (topic, deserialized message, bag timestamp ns) records."""
    counts = Counter()
    stamps = defaultdict(list)
    frames = Counter()
    sectors = {name: Counter() for name in ('front', 'left', 'rear', 'right')}
    minima = {}
    static_edges, dynamic_edges = set(), set()
    moving = 0
    max_linear, max_angular = 0.0, 0.0
    costmaps = []
    failures = Counter()
    for topic, msg, timestamp in records:
        counts[topic] += 1
        stamps[topic].append(timestamp / 1e9)
        if topic == '/scan':
            frames[msg.header.frame_id] += 1
            for index, value in enumerate(msg.ranges):
                angle = math.atan2(
                    math.sin(msg.angle_min + index * msg.angle_increment),
                    math.cos(msg.angle_min + index * msg.angle_increment))
                degrees = math.degrees(angle)
                name = ('front' if abs(degrees) <= 30 else
                        'rear' if abs(degrees) >= 150 else
                        'left' if degrees > 0 else 'right')
                valid = math.isfinite(value) and msg.range_min <= value <= msg.range_max
                sectors[name]['rays'] += 1
                sectors[name]['finite_in_range'] += int(valid)
                if valid:
                    minima[name] = min(minima.get(name, value), value)
        elif topic in ('/tf', '/tf_static'):
            edges = static_edges if topic == '/tf_static' else dynamic_edges
            for transform in msg.transforms:
                edges.add((transform.header.frame_id.lstrip('/'),
                           transform.child_frame_id.lstrip('/')))
        elif topic == '/cmd_vel_out':
            linear = math.hypot(msg.linear.x, msg.linear.y)
            angular = abs(msg.angular.z)
            moving += int(linear > 0.001 or angular > 0.001)
            max_linear = max(max_linear, linear)
            max_angular = max(max_angular, angular)
        elif topic == '/local_costmap/costmap':
            n = len(msg.data)
            costmaps.append({
                'frame': msg.header.frame_id, 'width': msg.info.width,
                'height': msg.info.height, 'resolution': msg.info.resolution,
                'unknown_pct': round(100 * sum(v < 0 for v in msg.data) / n, 2) if n else None,
            })
        elif topic == '/rosout' and any(word in msg.msg.lower() for word in (
                'collision ahead', 'failed to make progress', 'no valid trajectories',
                'hits unknown', 'oscillating', 'frame_id')):
            failures[f'{msg.name}: {msg.msg}'] += 1
    topics = {}
    for topic, count in counts.items():
        times = sorted(stamps[topic])
        duration = times[-1] - times[0]
        topics[topic] = {
            'messages': count,
            'average_hz': round((count - 1) / duration, 3) if duration > 0 else None,
            'largest_gap_s': round(max((b - a for a, b in zip(times, times[1:])), default=0), 3),
        }
    parents = defaultdict(set)
    for parent, child in static_edges | dynamic_edges:
        parents[child].add(parent)
    return {
        'topics': topics, 'scan_frames': dict(frames),
        'scan_sectors': {name: {
            **dict(values), 'nearest_return_m': minima.get(name),
        } for name, values in sectors.items()},
        'static_tf_edges': sorted(static_edges), 'dynamic_tf_edges': sorted(dynamic_edges),
        'tf_edges_on_both_topics': sorted(static_edges & dynamic_edges),
        'children_with_multiple_parents': {
            child: sorted(values) for child, values in parents.items() if len(values) > 1},
        'velocity_output': {
            'nonzero_messages': moving, 'max_linear_mps': max_linear,
            'max_abs_angular_rps': max_angular},
        'last_local_costmap': costmaps[-1] if costmaps else None,
        'local_costmap_messages': len(costmaps), 'failure_log_counts': dict(failures),
        'limitations': [
            'Topic rates use bag receipt time, not sensor timestamps.',
            'No-return rays are not proof of obstacles or sensor failure.',
            'A TF edge on both topics needs review; publisher identity is not available here.',
            'Costmap unknown percentage covers the entire grid, not just the robot footprint.',
            'This summary does not establish the root cause or navigation safety.',
        ],
    }


def bag_records(directory):
    """Read a bag offline. ROS imports are deliberately limited to this path."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(directory), storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    wanted = {'/scan', '/tf', '/tf_static', '/cmd_vel_out', '/local_costmap/costmap', '/rosout'}
    types = {entry.name: get_message(entry.type)
             for entry in reader.get_all_topics_and_types() if entry.name in wanted}
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic in types:
            yield topic, deserialize_message(data, types[topic]), timestamp


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('results_directory', type=Path)
    args = parser.parse_args()
    bags = sorted(args.results_directory.rglob('metadata.yaml'))
    if not bags:
        parser.error('No rosbag metadata.yaml found. Record the next smoke run with -RecordBag.')
    for metadata in bags:
        result = summarize(bag_records(metadata.parent))
        output = metadata.parent.parent / 'navigation_diagnostic.json'
        output.write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
        print(f'Wrote {output}')


if __name__ == '__main__':
    main()
