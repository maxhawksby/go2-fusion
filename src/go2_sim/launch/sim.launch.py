# Simulated Oak: Gazebo Fortress + the real robot stack (robot.launch.py with
# its driver swapped for go2_sim's sim_driver).
#
#   ros2 launch go2_sim sim.launch.py gui:=true explore:=true    # watch one run
#   ros2 launch go2_sim sim.launch.py                            # headless
#
# See docs/wiki/simulator.md for every argument.

import os
import shutil
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler,
                            SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from go2_sim import world_tools

ARGS = [
    ('world', 'placeholder_rooms',
     'World name in go2_sim/worlds, or a path to an .sdf file'),
    ('x', '2.5', 'Start x in the world (m)'),
    ('y', '2.0', 'Start y in the world (m)'),
    ('yaw', '0.0', 'Start heading in the world (rad)'),
    ('gui', 'false', 'Open the Gazebo window (false = headless server only)'),
    ('rviz', '', 'Open RViz (default: same as gui)'),
    ('explore', 'false', 'Start the auto mapper right away'),
    ('lidar_rate', '5.0', 'Lidar scans per second (real Oak: about 3-7)'),
    ('lidar_range', '12.0', 'Lidar maximum range (m)'),
    ('lidar_samples', '720', 'Lidar points per ring (16 rings); fewer leaves holes in /scan'),
    ('lidar_noise', '0.01', 'Lidar range noise, standard deviation (m)'),
    ('camera_rate', '', 'Front camera pictures per second on /camera/image_raw; 0 = no '
     'camera (default: 5 with vlm_mode on, else 0)'),
    ('vlm_mode', 'off', "The auto mapper's VLM frontier ranking: off, shadow or active"),
    ('vlm_url', '', 'Override: VLM chat completions URL'),
    ('vlm_model', '', 'Override: VLM model name'),
    ('params_file', '', 'sim_driver dials (default: go2_sim/config/sim.yaml)'),
    ('odom_noise_xy', '', 'Override: odometry noise, m per sqrt(m)'),
    ('odom_noise_yaw', '', 'Override: odometry noise, rad per sqrt(rad)'),
    ('odom_drift_yaw', '', 'Override: heading drift, rad per m'),
    ('cmd_latency', '', 'Override: command latency (s)'),
    ('seed', '0', 'Odometry noise seed (0 = random)'),
    ('domain_id', '42', 'ROS_DOMAIN_ID for the whole sim, kept apart from a real '
     'robot stack; empty = leave the current one'),
]


def _arg(context, name):
    return LaunchConfiguration(name).perform(context)


def _resolve_world(share, world):
    if world.endswith('.sdf') or os.sep in world:
        path = os.path.abspath(os.path.expanduser(world))
    else:
        path = os.path.join(share, 'worlds', world + '.sdf')
    if not os.path.isfile(path):
        raise RuntimeError(f'World not found: {path}')
    return path


def _setup(context):
    share = get_package_share_directory('go2_sim')
    gui = _arg(context, 'gui').lower() == 'true'
    rviz_arg = _arg(context, 'rviz').lower()
    rviz = gui if rviz_arg == '' else rviz_arg == 'true'

    if shutil.which('ign') is None:
        raise RuntimeError("Gazebo Fortress's `ign` command is missing. Rebuild the "
                           'image (docker compose build): it installs ros-humble-ros-gz.')

    # Environment for everything below. robot.launch.py reads these when it is
    # included: one robot (not robot0/...), no driver connection, no .ply saves.
    domain = _arg(context, 'domain_id')
    if domain:
        os.environ['ROS_DOMAIN_ID'] = domain
    os.environ['ROS_LOCALHOST_ONLY'] = '1'   # never reach a robot on the network
    os.environ['ROBOT_IP'] = 'sim'
    os.environ['CONN_TYPE'] = 'webrtc'
    os.environ['MAP_SAVE'] = 'false'
    os.environ.setdefault('IGN_PARTITION', f'go2_sim_{os.getpid()}')

    world_path = _resolve_world(share, _arg(context, 'world'))
    resource = [os.path.join(share, 'models'), os.path.dirname(world_path),
                os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')]
    os.environ['IGN_GAZEBO_RESOURCE_PATH'] = os.pathsep.join(p for p in resource if p)

    params_file = _arg(context, 'params_file') or os.path.join(share, 'config', 'sim.yaml')
    base_height = 0.32
    vlm_mode = _arg(context, 'vlm_mode')
    camera_rate = _arg(context, 'camera_rate') or ('0' if vlm_mode == 'off' else '5')
    with open(os.path.join(share, 'models', 'oak', 'model.sdf.template')) as f:
        robot = world_tools.render_robot(
            f.read(), base_height=base_height,
            lidar_rate=float(_arg(context, 'lidar_rate')),
            lidar_range=float(_arg(context, 'lidar_range')),
            lidar_samples=int(_arg(context, 'lidar_samples')),
            lidar_noise=float(_arg(context, 'lidar_noise')),
            camera_rate=float(camera_rate))
    with open(world_path) as f:
        world_text = f.read()
    pose = (float(_arg(context, 'x')), float(_arg(context, 'y')),
            base_height + 0.005, float(_arg(context, 'yaw')))
    composed = world_tools.compose_world(world_text, robot, pose)
    out_dir = tempfile.mkdtemp(prefix='go2_sim_')
    composed_path = os.path.join(out_dir, os.path.basename(world_path))
    with open(composed_path, 'w') as f:
        f.write(composed)

    # The server always runs headless, exactly as in the trials; the window
    # (gui:=true) is a separate viewer on top. On a Mac, windows go through
    # docker/mac's mac-gui wrapper (VirtualGL to XQuartz).
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-s', '-r', '-v', '2', '--headless-rendering', composed_path],
        output='screen')
    window = ['mac-gui'] if shutil.which('mac-gui') else []

    overrides = {'lidar_range': float(_arg(context, 'lidar_range')),
                 'base_height': base_height,
                 'seed': int(_arg(context, 'seed'))}
    for name in ('odom_noise_xy', 'odom_noise_yaw', 'odom_drift_yaw', 'cmd_latency'):
        value = _arg(context, name)
        if value:
            overrides[name] = float(value)

    # os.environ (above) is what robot.launch.py reads; these make sure every
    # process launch starts gets the same values whatever launch version runs.
    env_names = ['ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY', 'ROBOT_IP', 'CONN_TYPE', 'MAP_SAVE',
                 'IGN_PARTITION', 'IGN_GAZEBO_RESOURCE_PATH']
    actions = [SetEnvironmentVariable(n, os.environ[n]) for n in env_names if n in os.environ]
    actions += [
        gazebo,
        # Stop everything when Gazebo exits (window closed or crash).
        RegisterEventHandler(OnProcessExit(
            target_action=gazebo, on_exit=[EmitEvent(event=Shutdown(reason='Gazebo exited'))])),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='go2_sim_bridge',
             parameters=[{'config_file': os.path.join(share, 'config', 'bridge.yaml')}],
             output='screen'),
        Node(package='go2_sim', executable='sim_driver', name='go2_sim_driver',
             parameters=[params_file, overrides], output='screen'),
        # The real stack, unchanged, minus the driver and the extras a sim
        # does not need. use_sim_time stays false, as on the robot.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('go2_robot_sdk'), 'launch', 'robot.launch.py')),
            launch_arguments={'driver': 'false', 'rviz2': 'false', 'foxglove': 'false',
                              'joystick': 'false', 'teleop': 'true',
                              'slam': 'true', 'nav2': 'true'}.items()),
    ]
    if gui:
        viewer = ExecuteProcess(cmd=window + ['ign', 'gazebo', '-g', '-v', '1'], output='screen')
        actions += [viewer, RegisterEventHandler(OnProcessExit(
            target_action=viewer,
            on_exit=[EmitEvent(event=Shutdown(reason='Gazebo window closed'))]))]
    if rviz:
        actions.append(ExecuteProcess(
            cmd=window + ['rviz2', '-d', os.path.join(
                get_package_share_directory('go2_robot_sdk'), 'config',
                'single_robot_conf.rviz')],
            output='screen'))
    if _arg(context, 'explore').lower() == 'true':
        explorer_params = {'vlm_mode': vlm_mode}
        for name in ('vlm_url', 'vlm_model'):
            if _arg(context, name):
                explorer_params[name] = _arg(context, name)
        actions.append(Node(package='go2_auto_mapper', executable='explorer',
                            name='frontier_explorer', parameters=[explorer_params],
                            output='screen'))
    print(f'[go2_sim] world {world_path}, start {pose[0]:.2f} {pose[1]:.2f} '
          f'yaw {pose[3]:.2f}, ROS_DOMAIN_ID={os.environ.get("ROS_DOMAIN_ID", "0")}, '
          f'{"window" if gui else "headless"}')
    return actions


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=d, description=h) for n, d, h in ARGS]
        + [OpaqueFunction(function=_setup)])
