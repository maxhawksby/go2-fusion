from pathlib import Path

import rosbag2_py
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.serialization import serialize_message

from go2_interfaces.msg import AutoMapperEvent, ScoreboardStatus
from go2_scoreboard.replay import replay_bag


def _topic(writer, name, type_name):
    writer.create_topic(rosbag2_py.TopicMetadata(
        name=name, type=type_name, serialization_format='cdr'))


def _event(name, sequence, seconds):
    message = AutoMapperEvent()
    message.stamp.sec = seconds
    message.event_type = name
    message.goal_sequence = sequence
    return message


def _write_synthetic_bag(path):
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    _topic(writer, '/auto_mapper/events', 'go2_interfaces/msg/AutoMapperEvent')
    _topic(writer, '/odom', 'nav_msgs/msg/Odometry')
    _topic(writer, '/map', 'nav_msgs/msg/OccupancyGrid')
    _topic(writer, '/scoreboard/status', 'go2_interfaces/msg/ScoreboardStatus')
    prestart = Odometry()
    prestart.header.stamp.nanosec = 500_000_000
    prestart.pose.pose.position.x = -10.0
    writer.write('/odom', serialize_message(prestart), 500_000_000)
    for message, timestamp in (
        (_event('exploration_started', 0, 1), 1_000_000_000),
        (_event('goal_sent', 1, 2), 2_000_000_000),
        (_event('goal_reached', 1, 4), 4_000_000_000),
        (_event('exploration_completed', 1, 5), 5_000_000_000),
    ):
        writer.write('/auto_mapper/events', serialize_message(message), timestamp)
    for x, seconds in ((0.0, 1), (0.5, 2), (1.0, 3)):
        odom = Odometry()
        odom.header.stamp.sec = seconds
        odom.pose.pose.position.x = x
        writer.write('/odom', serialize_message(odom), seconds * 1_000_000_000)
    grid = OccupancyGrid()
    grid.header.stamp.sec = 4
    grid.info.width = 2
    grid.info.height = 2
    grid.info.resolution = 0.5
    grid.data = [0, 0, 100, -1]
    writer.write('/map', serialize_message(grid), 4_000_000_000)
    finalizing = ScoreboardStatus()
    finalizing.stamp.sec = 4
    finalizing.stamp.nanosec = 500_000_000
    finalizing.state = 'FINALIZING'
    writer.write('/scoreboard/status', serialize_message(finalizing), 4_500_000_000)
    postrun = Odometry()
    postrun.header.stamp.sec = 4
    postrun.header.stamp.nanosec = 800_000_000
    postrun.pose.pose.position.x = 100.0
    writer.write('/odom', serialize_message(postrun), 4_800_000_000)


def test_synthetic_bag_replay_is_deterministic(tmp_path):
    bag = tmp_path / 'bag'
    _write_synthetic_bag(bag)
    first, first_dir = replay_bag(str(bag), str(tmp_path / 'first'))
    second, second_dir = replay_bag(str(bag), str(tmp_path / 'second'))

    stable_fields = (
        'duration_s', 'distance_m', 'known_area_m2', 'goals_sent',
        'goals_reached', 'goals_failed', 'collisions', 'end_reason',
    )
    assert {key: first[key] for key in stable_fields} == {
        key: second[key] for key in stable_fields}
    assert first['distance_m'] == 1.0
    assert first['known_area_m2'] == 0.75
    assert first['goals_reached'] == 1
    assert Path(first_dir, 'final_map.yaml').is_file()
    assert Path(second_dir, 'scorecard.csv').is_file()
