"""The simulated Oak must look like the real one to the rest of the stack.

Runs with plain pytest and numpy, no ROS: sim_driver.py is loaded with fake
ROS modules (swapped in only while it imports, so the real rclpy in the
Humble container is left alone), and the real driver and launch files are
read as source with ast.
"""

import ast
import importlib.util
import math
import os
import sys
import types
from unittest import mock

import numpy as np
import pytest

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
SIM_DRIVER = os.path.join(REPO, 'src/go2_sim/go2_sim/sim_driver.py')
SIM_LAUNCH = os.path.join(REPO, 'src/go2_sim/launch/sim.launch.py')
ROBOT_LAUNCH = os.path.join(REPO, 'src/go2_robot_sdk/launch/robot.launch.py')
REAL_NODE = os.path.join(REPO, 'src/go2_robot_sdk/go2_robot_sdk/presentation/go2_driver_node.py')
REAL_PUBLISHER = os.path.join(
    REPO, 'src/go2_robot_sdk/go2_robot_sdk/infrastructure/ros2/ros2_publisher.py')

# The driver switches sim.launch.py may turn off; any other argument it passes
# must equal robot.launch.py's own default.
SIM_ONLY_SWITCHES = {'driver', 'rviz2', 'foxglove', 'joystick'}


# ------------------------------------------------------------ fake ROS

class Msg:
    """A message whose unset fields appear on first use; numbers start at 0."""

    FLOAT32 = 7

    def __init__(self, *_args, **fields):
        self.__dict__.update(fields)

    def __getattr__(self, name):
        if name.startswith('__'):
            raise AttributeError(name)
        value = 0.0 if name in ('x', 'y', 'z', 'w') else Msg()
        setattr(self, name, value)
        return value


class Time:
    def __init__(self, ns):
        self.nanoseconds = ns

    def __sub__(self, other):
        return Time(self.nanoseconds - other.nanoseconds)

    def to_msg(self):
        return Msg(sec=self.nanoseconds // 10**9, nanosec=self.nanoseconds % 10**9)


class Publisher:
    def __init__(self, msg_type, topic, qos):
        self.msg_type, self.topic, self.qos, self.sent = msg_type, topic, qos, []

    def publish(self, msg):
        self.sent.append(msg)


class FakeNode:
    parameter_overrides = {}

    def __init__(self, name):
        self.name = name
        self.now_ns = 10**9
        self.publishers = []

    def declare_parameter(self, name, default):
        return types.SimpleNamespace(value=self.parameter_overrides.get(name, default))

    def create_publisher(self, msg_type, topic, qos, **_kwargs):
        self.publishers.append(Publisher(msg_type, topic, qos))
        return self.publishers[-1]

    def create_subscription(self, *_args, **_kwargs):
        return None

    def create_timer(self, *_args, **_kwargs):
        return None

    def get_clock(self):
        return types.SimpleNamespace(now=lambda: Time(self.now_ns))

    def get_logger(self):
        return types.SimpleNamespace(info=lambda *_: None, warn=lambda *_: None)

    def publisher(self, topic):
        return next(p for p in self.publishers if p.topic == topic)


class Broadcaster:
    def __init__(self, _node, **_kwargs):
        self.sent = []

    def sendTransform(self, transform):
        self.sent.append(transform)


def _msg_type(name):
    return type(name, (Msg,), {})


def _fake_ros_modules():
    def module(name, **attrs):
        mod = types.ModuleType(name)
        mod.__dict__.update(attrs)
        return mod

    return {
        'rclpy': module('rclpy', init=lambda **_: None, ok=lambda: False,
                        spin=lambda _: None, shutdown=lambda: None),
        'rclpy.executors': module('rclpy.executors', ExternalShutdownException=Exception),
        'rclpy.node': module('rclpy.node', Node=FakeNode),
        'rclpy.qos': module(
            'rclpy.qos', QoSProfile=lambda **kw: dict(kw),
            ReliabilityPolicy=types.SimpleNamespace(BEST_EFFORT='BEST_EFFORT',
                                                    RELIABLE='RELIABLE')),
        'geometry_msgs': module('geometry_msgs'),
        'geometry_msgs.msg': module('geometry_msgs.msg', TransformStamped=_msg_type('TransformStamped'),
                                    Twist=_msg_type('Twist')),
        'nav_msgs': module('nav_msgs'),
        'nav_msgs.msg': module('nav_msgs.msg', Odometry=_msg_type('Odometry')),
        'sensor_msgs': module('sensor_msgs'),
        'sensor_msgs.msg': module('sensor_msgs.msg', JointState=_msg_type('JointState'),
                                  PointCloud2=_msg_type('PointCloud2'), PointField=Msg),
        'tf2_ros': module('tf2_ros', TransformBroadcaster=Broadcaster),
    }


def load_sim_driver():
    with mock.patch.dict(sys.modules, _fake_ros_modules()):
        spec = importlib.util.spec_from_file_location('sim_driver_under_test', SIM_DRIVER)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
    return mod


def make_driver(**parameters):
    mod = load_sim_driver()
    FakeNode.parameter_overrides = {'seed': 1, **parameters}
    try:
        return mod.SimDriver()
    finally:
        FakeNode.parameter_overrides = {}


def truth(x, speed):
    """Gazebo's ground-truth odometry: Oak at (x, 0) heading +x at `speed` m/s."""
    msg = Msg()
    msg.pose.pose.position.x = x
    msg.pose.pose.orientation.w = 1.0
    msg.twist.twist.linear.x = speed
    msg.twist.twist.angular.z = 0.3
    return msg


def walk(driver, speed=0.4):
    driver.on_truth(truth(0.0, speed))
    driver.now_ns += 10**8
    driver.on_truth(truth(speed * 0.1, speed))
    return driver.publisher('odom').sent[-1]


# ------------------------------------------------------------ source facts

def parse(path):
    with open(path) as f:
        return ast.parse(f.read(), filename=path)


def call_name(node):
    func = node.func
    return func.attr if isinstance(func, ast.Attribute) else getattr(func, 'id', None)


def robot_launch_defaults():
    defaults = {}
    for node in ast.walk(parse(ROBOT_LAUNCH)):
        if isinstance(node, ast.Call) and call_name(node) == 'DeclareLaunchArgument':
            name = ast.literal_eval(node.args[0])
            for kw in node.keywords:
                if kw.arg == 'default_value':
                    defaults[name] = ast.literal_eval(kw.value)
    return defaults


def sim_launch_include():
    for node in ast.walk(parse(SIM_LAUNCH)):
        if isinstance(node, ast.Call) and call_name(node) == 'IncludeLaunchDescription':
            files = [c.value for c in ast.walk(node)
                     if isinstance(c, ast.Constant) and isinstance(c.value, str)]
            for kw in node.keywords:
                if kw.arg == 'launch_arguments':
                    return files, ast.literal_eval(kw.value.func.value)
    raise AssertionError('sim.launch.py includes no launch file')


def real_driver_publishers():
    """{message type: (topic, reliability, depth)} for the real driver, one robot."""
    tree = parse(REAL_NODE)
    setup = next(n for n in ast.walk(tree)
                 if isinstance(n, ast.FunctionDef) and n.name == '_setup_publishers')
    qos = {}
    for node in ast.walk(setup):
        if isinstance(node, ast.Assign) and isinstance(node.value, ast.Call) \
                and call_name(node.value) == 'QoSProfile':
            kws = {kw.arg: kw.value for kw in node.value.keywords}
            reliability = kws['reliability'].attr if 'reliability' in kws else 'RELIABLE'
            qos[node.targets[0].id] = (reliability, ast.literal_eval(kws['depth']))
    single = next(n for n in ast.walk(setup) if isinstance(n, ast.If)
                  and "'single'" in ast.unparse(n.test))
    topics = {n.targets[0].id: n.value.value for n in single.body
              if isinstance(n, ast.Assign) and isinstance(n.value, ast.Constant)}
    out = {}
    for node in ast.walk(setup):
        if isinstance(node, ast.Call) and call_name(node) == 'create_publisher':
            msg_type, topic, profile = (a.id for a in node.args[:3])
            out[msg_type] = (topics[topic], *qos[profile])
    return out


def real_frame_ids(function):
    tree = parse(REAL_PUBLISHER)
    body = [n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == function]
    ids = set()
    for fn in body:
        for node in ast.walk(fn):
            if isinstance(node, ast.Assign) and isinstance(node.value, ast.Constant) and \
                    isinstance(node.targets[0], ast.Attribute) and \
                    node.targets[0].attr in ('frame_id', 'child_frame_id'):
                ids.add((node.targets[0].attr, node.value.value))
            if isinstance(node, ast.keyword) and node.arg == 'frame_id' and \
                    isinstance(node.value, ast.Constant):
                ids.add(('frame_id', node.value.value))
    return ids


def sim_qos(publisher):
    qos = publisher.qos
    if isinstance(qos, int):
        return ('RELIABLE', qos)
    return (qos.get('reliability', 'RELIABLE'), qos['depth'])


# ------------------------------------------------------------ tests

def test_sim_odom_reports_zero_speed_like_oak():
    """Oak's driver never fills /odom's twist (ros2_publisher.py), so Nav2 always
    reads 0. The stand-in must too, or goal approach differs from Oak's."""
    odom = walk(make_driver())
    twist = odom.twist.twist
    assert (twist.linear.x, twist.linear.y, twist.angular.z) == (0.0, 0.0, 0.0), \
        f'sim /odom carries speed {twist.linear.x} m/s, {twist.angular.z} rad/s; Oak sends 0'


def test_sim_odom_true_speed_is_opt_in():
    odom = walk(make_driver(odom_true_speed=True), speed=0.4)
    assert math.isclose(odom.twist.twist.linear.x, 0.4)
    assert math.isclose(odom.twist.twist.angular.z, 0.3)


def test_sim_odom_pose_still_moves():
    odom = walk(make_driver(odom_noise_xy=0.0, odom_noise_yaw=0.0), speed=0.4)
    assert math.isclose(odom.pose.pose.position.x, 0.04, abs_tol=1e-9)


def test_sim_launch_runs_robot_launch_with_only_driver_extras_off():
    files, arguments = sim_launch_include()
    assert 'robot.launch.py' in files
    defaults = robot_launch_defaults()
    for switch in SIM_ONLY_SWITCHES:
        assert arguments.get(switch) == 'false', f'sim.launch.py should turn off {switch}'
    drift = {k: v for k, v in arguments.items()
             if k not in SIM_ONLY_SWITCHES and defaults.get(k) != v}
    assert drift == {}, f'sim.launch.py changes robot.launch.py arguments Oak runs with: {drift}'


def test_sim_driver_publishes_real_driver_topic_names_and_reliability():
    real = real_driver_publishers()
    sim = make_driver()
    for msg_type in ('Odometry', 'JointState', 'PointCloud2'):
        pub = next(p for p in sim.publishers if p.msg_type.__name__ == msg_type)
        topic, reliability, _depth = real[msg_type]
        assert pub.topic == topic, msg_type
        assert sim_qos(pub)[0] == reliability, msg_type


def test_sim_driver_publisher_queue_depths_match_real_driver():
    real = real_driver_publishers()
    sim = make_driver()
    for msg_type in ('Odometry', 'JointState', 'PointCloud2'):
        pub = next(p for p in sim.publishers if p.msg_type.__name__ == msg_type)
        assert sim_qos(pub)[1] == real[msg_type][2], \
            f'{msg_type} on {pub.topic}: sim depth {sim_qos(pub)[1]}, Oak {real[msg_type][2]}'


def test_sim_frame_ids_match_real_driver():
    driver = make_driver()
    odom = walk(driver)
    transform = driver.tf.sent[-1]
    real_odom = real_frame_ids('_publish_odometry_topic') | real_frame_ids('_publish_transform')
    for msg in (odom, transform):
        assert ('frame_id', msg.header.frame_id) in real_odom
        assert ('child_frame_id', msg.child_frame_id) in real_odom

    points = np.array([[2.0, 0.0, 0.0]], dtype='<f4')
    raw = Msg(width=1, height=1, point_step=12, data=points.tobytes(),
              fields=[Msg(name=k, offset=4 * i) for i, k in enumerate('xyz')])
    driver.on_points(raw)
    cloud = driver.publisher('point_cloud2').sent[-1]
    assert ('frame_id', cloud.header.frame_id) in real_frame_ids('publish_lidar_data')
    assert [f.name for f in cloud.fields] == ['x', 'y', 'z', 'intensity']


if __name__ == '__main__':
    sys.exit(pytest.main([__file__, '-v']))
