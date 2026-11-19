"""Opt-in Gazebo angle check; domain 88/local-only, no WebRTC driver or model call.

Run only after reviewing the test scope. Logs and JSON results stay under /tmp.
Reuses go2_sim's model and simulated driver; teammate source files are not edited.
"""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--noise-yaw', type=float, default=0.0)
    parser.add_argument('--latency', type=float, default=0.1)
    parser.add_argument('--seed', type=int, default=17)
    args = parser.parse_args()
    if os.getenv('ROS_DOMAIN_ID') != '88' or os.getenv('ROS_LOCALHOST_ONLY') != '1':
        raise SystemExit('Requires ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1; never use the robot domain.')
    if any(not math.isfinite(v) or v < 0 for v in (args.noise_yaw, args.latency)):
        raise SystemExit('Noise and latency must be finite and nonnegative.')

    # ROS imports and all process creation happen after the isolation check.
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from go2_sim import world_tools
    from go2_llm_agent.agent import AgentSession
    from go2_llm_agent.commands import parse_simple_request
    from go2_llm_agent.heading import HeadingTracker
    from go2_llm_agent.ros2_motion import RosMotionController

    output = Path(tempfile.mkdtemp(prefix='go2-gazebo-turn-check-'))
    share = Path(get_package_share_directory('go2_sim'))
    robot = world_tools.render_robot((share / 'models/oak/model.sdf.template').read_text())
    world = '''<sdf version="1.8"><world name="angle_check">
      <physics name="4ms" type="ignored"><max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor></physics>
      <model name="ground"><static>true</static><link name="floor">
      <collision name="floor"><geometry><plane><normal>0 0 1</normal>
      <size>40 40</size></plane></geometry></collision></link></model>
      </world></sdf>'''
    composed = world_tools.compose_world(world, robot, (0, 0, .325, math.radians(170)))
    root = ET.fromstring(composed)
    # Only heading is tested: no rendering, lidar, SLAM, Nav2, speech or windows.
    for parent in root.iter():
        for child in list(parent):
            if child.tag == 'sensor' or (child.tag == 'plugin' and 'sensors-system' in child.get('filename', '')):
                parent.remove(child)
    world_file = output / 'world.sdf'
    ET.ElementTree(root).write(world_file, encoding='unicode')
    env = dict(os.environ, ROBOT_IP='sim', IGN_IP='127.0.0.1',
               IGN_PARTITION=f'go2_angle_check_{os.getpid()}')
    processes, handles, results = [], [], []
    motion, session, probe = None, None, None

    def start(name, command):
        handle = (output / (name + '.log')).open('w')
        handles.append(handle)
        processes.append(subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT,
                                          env=env, start_new_session=True))

    class Client:
        def interpret(self, text):
            plan = parse_simple_request(text)
            if plan is None or plan.status != 'execute':
                raise RuntimeError('Test command was not recognized; no model fallback is allowed.')
            return plan

    def interrupted(signum, frame):
        raise KeyboardInterrupt

    old_term = signal.signal(signal.SIGTERM, interrupted)
    rclpy.init()
    try:
        probe = Node('go2_llm_gazebo_probe')
        truth = HeadingTracker()
        odom = HeadingTracker()

        def receive(msg, tracker):
            q = msg.pose.pose.orientation
            tracker.update((q.x, q.y, q.z, q.w), (msg.header.frame_id, msg.child_frame_id))

        probe.create_subscription(Odometry, '/sim/ground_truth/odom',
                                  lambda msg: receive(msg, truth), qos_profile_sensor_data)
        probe.create_subscription(Odometry, '/odom',
                                  lambda msg: receive(msg, odom), qos_profile_sensor_data)
        # No existing session may share this test's topics.
        until = time.monotonic() + 2
        while time.monotonic() < until:
            rclpy.spin_once(probe, timeout_sec=.05)
        if probe.get_publishers_info_by_topic('/odom') or probe.get_publishers_info_by_topic('/cmd_vel_agent'):
            raise RuntimeError('Domain 88 is already in use; no simulator or movement was started.')

        start('gazebo', ['ign', 'gazebo', '-s', '-r', '-v', '1', str(world_file)])
        start('bridge', ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                         '/model/oak/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                         '/model/oak/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                         '--ros-args', '-r', '/model/oak/cmd_vel:=/sim/cmd_vel',
                         '-r', '/model/oak/odometry:=/sim/ground_truth/odom'])
        start('sim_driver', ['ros2', 'run', 'go2_sim', 'sim_driver', '--ros-args',
                             '-p', f'odom_noise_yaw:={args.noise_yaw}', '-p', 'odom_noise_xy:=0.0',
                             '-p', f'cmd_latency:={args.latency}', '-p', f'seed:={args.seed}'])
        mux_file = Path(get_package_share_directory('go2_robot_sdk')) / 'config/twist_mux.yaml'
        start('mux', ['ros2', 'run', 'twist_mux', 'twist_mux', '--ros-args', '--params-file', str(mux_file)])
        motion = RosMotionController()
        motion.start()

        until = time.monotonic() + 25
        while time.monotonic() < until:
            rclpy.spin_once(probe, timeout_sec=.05)
            if any(process.poll() is not None for process in processes):
                raise RuntimeError(f'A simulator process exited. Logs: {output}')
            snapshots = (motion.read_heading(), odom.read(), truth.read())
            if (all(sample is not None and time.monotonic() - sample.received_at < .5
                    for sample in snapshots)
                    and probe.count_subscribers("/cmd_vel_agent") == 1):
                break
        else:
            raise RuntimeError(f'Simulator odometry did not start. Logs: {output}')
        publishers = probe.get_publishers_info_by_topic('/odom')
        if len(publishers) != 1 or publishers[0].node_name != 'go2_sim_driver':
            raise RuntimeError('Unexpected odometry source; refusing to run commands.')
        print(f'Gazebo only, domain 88. Logs/results: {output}', flush=True)
        print(f'noise_yaw={args.noise_yaw}, command_latency={args.latency}; Qwen is bypassed.', flush=True)

        for direction, angle in (('left', 30), ('right', 90), ('left', 180), ('right', 360)):
            before_odom, before_truth = odom.read().radians, truth.read().radians
            session = AgentSession(Client(), motion)
            session.submit(f'turn {direction} {angle} degrees')
            until = time.monotonic() + 40
            while session._worker.is_alive() and time.monotonic() < until:
                rclpy.spin_once(probe, timeout_sec=.02)
            if session._worker.is_alive():
                session.stop()
                session.wait()
                raise RuntimeError('Test request exceeded its watchdog deadline.')
            session.wait()
            until = time.monotonic() + .5
            while time.monotonic() < until:
                rclpy.spin_once(probe, timeout_sec=.02)
            sign = 1 if direction == 'left' else -1
            measured = sign * math.degrees(odom.read().radians - before_odom)
            actual = sign * math.degrees(truth.read().radians - before_truth)
            result = dict(direction=direction, requested_deg=angle, odometry_deg=measured,
                          ground_truth_deg=actual, odometry_error_deg=measured-angle,
                          ground_truth_error_deg=actual-angle, error=str(session.last_error) if session.last_error else None)
            results.append(result)
            print(json.dumps(result), flush=True)
            if session.last_error or abs(measured-angle) > 1.1:
                raise RuntimeError('Measured turn did not meet the reported-heading tolerance.')
            if args.noise_yaw == 0 and abs(actual-angle) > 1.5:
                raise RuntimeError('Noiseless Gazebo turn missed its ground-truth tolerance.')
            session = None
    finally:
        if session is not None:
            session.close()
        elif motion is not None:
            motion.close()
        for process in processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        for handle in handles:
            handle.close()
        if probe is not None:
            probe.destroy_node()
        rclpy.shutdown()
        signal.signal(signal.SIGTERM, old_term)
        (output / 'results.json').write_text(json.dumps(
            dict(noise_yaw=args.noise_yaw, latency=args.latency, seed=args.seed, turns=results), indent=2)+'\n')
        print(f'Results saved: {output / "results.json"}', flush=True)


if __name__ == '__main__':
    main()
