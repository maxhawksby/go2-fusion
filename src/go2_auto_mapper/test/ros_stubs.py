"""Just enough fake ROS 2 modules to import go2_auto_mapper.explorer with
plain Python. load_explorer() always uses them, even where the real rclpy is
installed, so the same test runs on a bare runner and inside the Humble
container, and the other tests there still see the real ROS modules."""

import importlib.util
import sys
import types
from pathlib import Path


class Auto:
    """A message whose unset fields spring into existence on first use."""

    def __init__(self, *_args, **fields):
        self.__dict__.update(fields)

    def __getattr__(self, name):
        if name.startswith('__'):
            raise AttributeError(name)
        value = Auto()
        setattr(self, name, value)
        return value


class Twist:
    def __init__(self):
        self.linear = Auto(x=0.0, y=0.0, z=0.0)
        self.angular = Auto(x=0.0, y=0.0, z=0.0)


class Action:
    Goal = Auto


class Node:
    """Records what the explorer creates; the harness drives it."""

    def __init__(self, name):
        self._params = {}
        self._timers = []
        self._publishers = {}
        self.log = []

    def declare_parameter(self, name, value=None, *_):
        self._params.setdefault(name, value)

    def get_parameter(self, name):
        return Auto(value=self._params[name])

    def create_publisher(self, _type, topic, *_):
        publisher = Auto(topic=topic, sent=[])
        publisher.publish = publisher.sent.append
        self._publishers[topic] = publisher
        return publisher

    def create_subscription(self, *_):
        return Auto()

    def create_service(self, *_):
        return Auto()

    def create_timer(self, period, callback):
        self._timers.append([period, callback, period])
        return Auto()

    def get_logger(self):
        def at(level):
            return lambda msg, **_: self.log.append((level, msg))
        return Auto(info=at('info'), warn=at('warn'), error=at('error'),
                    debug=at('debug'))

    def get_clock(self):
        return self._clock


class TransformException(Exception):
    pass


def _install():
    def module(name, **attrs):
        mod = types.ModuleType(name)
        mod.__dict__.update(attrs)
        sys.modules[name] = mod
        return mod

    any_type = Auto
    rclpy = module('rclpy', init=lambda **_: None, ok=lambda: True)
    rclpy.node = module('rclpy.node', Node=Node)
    rclpy.action = module('rclpy.action', ActionClient=None)
    rclpy.executors = module('rclpy.executors',
                             ExternalShutdownException=Exception)
    rclpy.qos = module('rclpy.qos', QoSProfile=any_type, DurabilityPolicy=Auto(),
                       HistoryPolicy=Auto(), ReliabilityPolicy=Auto())
    rclpy.signals = module('rclpy.signals', SignalHandlerOptions=Auto())
    rclpy.time = module('rclpy.time', Time=any_type)
    rclpy.duration = module('rclpy.duration', Duration=any_type)
    module('rcl_interfaces')
    module('rcl_interfaces.msg', ParameterDescriptor=any_type)
    module('action_msgs')
    module('action_msgs.srv', CancelGoal=any_type)
    module('nav_msgs')
    module('nav_msgs.msg', OccupancyGrid=any_type, Path=any_type)
    module('std_srvs')
    module('std_srvs.srv', Trigger=any_type)
    module('geometry_msgs')
    module('geometry_msgs.msg', PoseStamped=any_type, Twist=Twist)
    module('sensor_msgs')
    module('sensor_msgs.msg', Image=any_type, LaserScan=any_type,
           PointCloud2=any_type)
    module('std_msgs')
    module('std_msgs.msg', String=any_type)
    module('go2_interfaces')
    module('go2_interfaces.msg', AutoMapperEvent=any_type)
    module('builtin_interfaces')
    module('builtin_interfaces.msg', Duration=any_type, Time=any_type)
    module('nav2_msgs')
    module('nav2_msgs.action', BackUp=type('BackUp', (Action,), {}),
           NavigateToPose=type('NavigateToPose', (Action,), {}),
           Spin=type('Spin', (Action,), {}))
    module('tf2_ros', Buffer=any_type, TransformListener=any_type,
           TransformException=TransformException)
    module('cv_bridge', CvBridge=any_type)


STUBBED = ('rclpy', 'rcl_interfaces', 'action_msgs', 'nav_msgs', 'std_srvs',
           'geometry_msgs', 'sensor_msgs', 'std_msgs', 'go2_interfaces',
           'builtin_interfaces', 'nav2_msgs', 'tf2_ros', 'cv_bridge')


def _stubbed(name):
    return name.split('.')[0] in STUBBED


def load_explorer():
    """A private copy of go2_auto_mapper.explorer built on these stubs."""

    package = Path(__file__).resolve().parents[1]
    if str(package) not in sys.path:
        sys.path.insert(0, str(package))
    saved = {name: mod for name, mod in sys.modules.items() if _stubbed(name)}
    for name in saved:
        del sys.modules[name]
    try:
        _install()
        spec = importlib.util.spec_from_file_location(
            'stubbed_explorer', package / 'go2_auto_mapper' / 'explorer.py')
        explorer = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(explorer)
        return explorer
    finally:
        for name in [n for n in sys.modules if _stubbed(n)]:
            del sys.modules[name]
        sys.modules.update(saved)
