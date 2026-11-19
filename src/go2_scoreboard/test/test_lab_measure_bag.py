import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import pytest
from rclpy.serialization import serialize_message
import rosbag2_py

from go2_interfaces.msg import IMU
from go2_scoreboard.lab_measure import Pose, main
from lab_samples import STOP_COMMANDS, STOP_POSES, TURN_COMMANDS, TURN_GYRO, TURN_POSES

TYPES = {
    '/cmd_vel_joy': 'geometry_msgs/msg/Twist',
    '/odom': 'nav_msgs/msg/Odometry',
    '/imu': 'go2_interfaces/msg/IMU',
    '/scan': 'sensor_msgs/msg/LaserScan',
    '/point_cloud2': 'sensor_msgs/msg/PointCloud2',
}
# Recordings start at wall-clock time, not zero.
BAG_START_NS = 1_700_000_000 * 10**9


def _twist(command):
    message = Twist()
    message.linear.x = command.linear
    message.angular.z = command.angular
    return message


def _odometry(pose, z=0.0):
    message = Odometry()
    message.pose.pose.position.x = pose.x
    message.pose.pose.position.y = pose.y
    message.pose.pose.position.z = z
    message.pose.pose.orientation.z = math.sin(pose.yaw / 2)
    message.pose.pose.orientation.w = math.cos(pose.yaw / 2)
    return message


def _imu(sample):
    message = IMU()
    message.gyroscope = [0.0, 0.0, sample.yaw_rate]
    return message


def write_bag(path, timed_messages):
    """Write (topic, seconds, message) rows as a rosbag2 folder, in time order."""
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    rows = sorted(timed_messages, key=lambda row: row[1])
    for topic in sorted({row[0] for row in rows}):
        writer.create_topic(rosbag2_py.TopicMetadata(
            name=topic, type=TYPES[topic], serialization_format='cdr'))
    for topic, seconds, message in rows:
        writer.write(topic, serialize_message(message), BAG_START_NS + round(seconds * 1e9))
    del writer


def stop_rows():
    return ([('/cmd_vel_joy', c.t, _twist(c)) for c in STOP_COMMANDS]
            + [('/odom', p.t, _odometry(p)) for p in STOP_POSES])


def turn_rows():
    return ([('/cmd_vel_joy', c.t, _twist(c)) for c in TURN_COMMANDS]
            + [('/odom', p.t, _odometry(p)) for p in TURN_POSES]
            + [('/imu', g.t, _imu(g)) for g in TURN_GYRO])


def object_rows():
    """Oak stands at (2, 1) facing +y in odom; a backpack appears 0.5 m ahead at 20 s."""
    from sensor_msgs.msg import LaserScan, PointCloud2, PointField

    oak = _odometry(Pose(0.0, 2.0, 1.0, math.pi / 2), z=0.3)
    rows = [('/odom', t / 10, oak) for t in range(300)]
    for second in range(30):
        scan = LaserScan()
        scan.header.frame_id = 'base_link'
        scan.angle_min = -math.pi
        scan.angle_increment = math.tau / 360
        scan.ranges = [math.inf] * 360
        cloud = PointCloud2()
        cloud.header.frame_id = 'odom'
        cloud.fields = [PointField(name=name, offset=4 * i, datatype=PointField.FLOAT32,
                                   count=1) for i, name in enumerate('xyz')]
        cloud.point_step = 16
        cloud.height = 1
        floor = [(2.0, 1.4 + 0.1 * i, 0.0) for i in range(8)]
        backpack = []
        if second >= 20:
            scan.ranges[180] = 0.5
            backpack = [(2.0, 1.5, 0.05 + 0.02 * i) for i in range(10)]
        points = np.zeros((len(floor) + len(backpack), 4), dtype='<f4')
        points[:, :3] = floor + backpack
        cloud.width = len(points)
        cloud.row_step = 16 * len(points)
        cloud.data = points.tobytes()
        rows += [('/scan', second + 0.5, scan), ('/point_cloud2', second + 0.5, cloud)]
    return rows


def test_stop_recording_prints_the_stop_distance(tmp_path, capsys):
    write_bag(tmp_path / 'lab_stop', stop_rows())

    code = main([str(tmp_path / 'lab_stop'), '--kind', 'stop'])

    report = capsys.readouterr().out
    assert code == 0
    assert '| 1 | 2.0 to 4.9 | 0.30 | 0.30 | 1.96 | 0.40 |' in report
    assert 'Recording: `lab_stop`' in report


def test_turn_recording_reads_oaks_own_imu_message(tmp_path, capsys):
    write_bag(tmp_path / 'lab_turn', turn_rows())

    code = main([str(tmp_path / 'lab_turn'), '--kind', 'turn'])

    assert code == 0
    assert '| 1 | 1.0 | 0.50 | 2.08 | 1.58 | 0.80 | 0.80 |' in capsys.readouterr().out


def test_turn_recording_without_imu_warns_and_still_reports(tmp_path, capsys):
    write_bag(tmp_path / 'lab_turn', [row for row in turn_rows() if row[0] != '/imu'])

    code = main([str(tmp_path / 'lab_turn'), '--kind', 'turn'])

    report = capsys.readouterr().out
    assert code == 0
    assert '| 1 | 1.0 | - | 2.08 | - | 0.80 | 0.80 |' in report
    assert '/imu is not in this recording' in report


def test_object_recording_sees_the_backpack_in_scan_and_cloud(tmp_path, capsys):
    pytest.importorskip('sensor_msgs')
    write_bag(tmp_path / 'lab_object', object_rows())

    code = main([str(tmp_path / 'lab_object'), '--kind', 'object'])

    report = capsys.readouterr().out
    assert code == 0
    assert '| 15 to 20 | 5 | - | - | 5 | 8.0 | 0.0 | 0.0 |' in report
    assert '| 20 to 25 | 5 | 0.50 | 0.50 | 5 | 8.0 | 10.0 | 0.0 |' in report


def test_recording_with_none_of_the_topics_exits_non_zero(tmp_path, capsys):
    write_bag(tmp_path / 'lab_stop', [('/odom', p.t, _odometry(p)) for p in STOP_POSES[:5]])

    code = main([str(tmp_path / 'lab_stop'), '--kind', 'stop'])

    report = capsys.readouterr().out
    assert code == 1
    assert 'No repeats found' in report
    assert '/cmd_vel_joy is not in this recording' in report
