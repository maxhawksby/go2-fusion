"""Stands in for go2_driver_node when Oak is simulated.

It speaks the real driver's topics, so twist_mux, SLAM, Nav2 and the auto
mapper run unchanged:

- listens on /cmd_vel_out (twist_mux's output), adds the command latency and
  forwards it to Gazebo's velocity control (/sim/cmd_vel);
- turns Gazebo's true pose into /odom and the odom -> base_link transform,
  adding odometry noise and drift (the real robot's step counting is not exact);
- moves Gazebo's lidar points into the odom frame and publishes /point_cloud2,
  like the real driver (which publishes its cloud in odom);
- publishes a standing /joint_states so robot_state_publisher draws the legs.

Everything is stamped with this node's clock when it is published, as the real
driver does, so the stack keeps use_sim_time false. Gazebo runs in real time.
"""

import math
from collections import deque

import numpy as np
import rclpy
import rclpy.executors
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, PointCloud2, PointField
from tf2_ros import TransformBroadcaster

JOINTS = [
    'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
    'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
    'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
    'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
]
STANDING = [0.0, 0.67, -1.3] * 4


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def oak_response(vx, vy, wz, u, min_walk_speed, min_turn_rate, sure_turn_rate,
                 unsure_turn_success, turn_gain):
    """The motion Oak makes for a velocity command, after its deadband and turn gain.

    Below min_walk_speed Oak stands still; standing, it ignores turns slower
    than min_turn_rate and makes turns below sure_turn_rate only when the
    uniform draw u is under unsure_turn_success. Walking, any turn works.
    """
    if math.hypot(vx, vy) < min_walk_speed:
        vx = vy = 0.0
        rate = abs(wz)
        if rate < min_turn_rate or (rate < sure_turn_rate and u >= unsure_turn_success):
            wz = 0.0
    return vx, vy, wz * turn_gain


def blind_points(xy, blind_sectors, sector_count):
    """True for each point whose azimuth (around base_link) falls in a blind sector."""
    az = np.arctan2(xy[:, 1], xy[:, 0])
    sector = ((az + math.pi) / (2 * math.pi) * sector_count).astype(int) % sector_count
    return np.isin(sector, blind_sectors)


def approach(current, target, max_step):
    if max_step <= 0:
        return target
    return current + max(-max_step, min(max_step, target - current))


class SimDriver(Node):

    def __init__(self):
        super().__init__('go2_sim_driver')
        p = self.declare_parameter
        # Odometry error (see docs/wiki/simulator.md for what each one does).
        self.noise_xy = p('odom_noise_xy', 0.01).value      # m per sqrt(m) travelled
        self.noise_yaw = p('odom_noise_yaw', 0.01).value    # rad per sqrt(rad) turned
        self.scale_xy = p('odom_scale_xy', 0.0).value       # 0.02 = reports 2 % too far
        self.scale_yaw = p('odom_scale_yaw', 0.0).value     # 0.02 = reports 2 % too much turn
        self.drift_yaw = p('odom_drift_yaw', 0.0).value     # rad of heading drift per m
        self.odom_rate = p('odom_rate', 15.0).value         # Hz, real Oak about 13
        # Oak's driver leaves /odom's twist at 0, and Nav2's DWB reads it.
        self.odom_true_speed = p('odom_true_speed', False).value
        self.base_height = p('base_height', 0.32).value     # base_link above the floor
        # Commands.
        self.latency = p('cmd_latency', 0.1).value          # s from /cmd_vel_out to motion
        self.cmd_timeout = p('cmd_timeout', 0.5).value      # stop if no command this long
        self.max_vx = p('max_vel_x', 1.0).value
        self.max_vy = p('max_vel_y', 0.6).value
        self.max_wz = p('max_vel_yaw', 2.0).value
        self.acc_xy = p('max_accel_xy', 1.5).value          # m/s^2, 0 = instant
        self.acc_yaw = p('max_accel_yaw', 3.0).value        # rad/s^2, 0 = instant
        # Oak's deadband and turn gain (sim.yaml has the measurements).
        self.min_walk_speed = p('min_walk_speed', 0.0).value
        self.min_turn_rate = p('min_turn_rate', 0.0).value
        self.sure_turn_rate = p('sure_turn_rate', 0.0).value
        self.unsure_turn_success = p('unsure_turn_success', 1.0).value
        self.turn_gain = p('turn_gain', 1.0).value
        # Lidar.
        self.lidar_mount = (p('lidar_x', 0.28945).value, p('lidar_y', 0.0).value,
                            p('lidar_z', -0.046825).value)
        self.lidar_range = p('lidar_range', 20.0).value
        # Points inside this box around base_link are Oak's own head and legs:
        # dropped, as on the real robot (its cloud has no points on its body).
        # Kept inside the 0.36 m nose so obstacles touching it stay visible.
        self.self_box = (p('self_filter_x', 0.32).value, p('self_filter_y', 0.18).value)
        # Oak's floor smear and blind directions (sim.yaml has the measurements).
        self.z_noise = p('lidar_z_noise', 0.0).value     # m, sd of each point's height
        self.blind_fraction = p('lidar_blind_fraction', 0.0).value
        self.blind_sector_count = int(p('lidar_blind_sectors', 36).value)
        self.blind_hold = p('lidar_blind_hold', 10.0).value  # s before new blind sectors
        seed = p('seed', 0).value
        self.rng = np.random.default_rng(seed if seed > 0 else None)

        self.tf = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.cloud_pub = self.create_publisher(
            PointCloud2, 'point_cloud2',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.gz_cmd_pub = self.create_publisher(Twist, '/sim/cmd_vel', 10)

        self.create_subscription(Twist, 'cmd_vel_out', self.on_cmd, 10)
        self.create_subscription(Odometry, '/sim/ground_truth/odom', self.on_truth, 50)
        self.create_subscription(
            PointCloud2, '/sim/lidar/points', self.on_points,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))

        self.pending = deque()          # (due time ns, Twist)
        self.target = (0.0, 0.0, 0.0)
        self.target_time = None
        self.out = [0.0, 0.0, 0.0]
        self.prev_truth = None          # (x, y, yaw)
        self.odom = [0.0, 0.0, 0.0]     # Oak's own estimate, starts at 0 like the real one
        self.odom_twist = (0.0, 0.0, 0.0)
        self.last_odom_pub = None
        # go2.urdf also has a fixed odom -> base_link joint at floor level. A
        # cloud sent before our odometry starts is placed with that one, so the
        # floor lands in /scan's band and SLAM draws rings of fake wall around
        # the start.
        self.odom_started = False
        self.blind = np.array([], dtype=int)
        self.blind_drawn_ns = None

        self.cmd_period = 0.02
        self.create_timer(self.cmd_period, self.step_cmd)
        self.create_timer(0.1, self.publish_joints)
        self.get_logger().info(
            f'Simulated Oak driver up: latency {self.latency:.2f} s, odometry noise '
            f'{self.noise_xy} m/sqrt(m), {self.noise_yaw} rad/sqrt(rad), '
            f'drift {self.drift_yaw} rad/m')

    # ------------------------------------------------------------- commands
    def on_cmd(self, msg):
        due = self.get_clock().now().nanoseconds + int(self.latency * 1e9)
        self.pending.append((due, msg))

    def step_cmd(self):
        now = self.get_clock().now().nanoseconds
        while self.pending and self.pending[0][0] <= now:
            _, msg = self.pending.popleft()
            self.target = oak_response(
                max(-self.max_vx, min(self.max_vx, msg.linear.x)),
                max(-self.max_vy, min(self.max_vy, msg.linear.y)),
                max(-self.max_wz, min(self.max_wz, msg.angular.z)),
                self.rng.random(), self.min_walk_speed, self.min_turn_rate,
                self.sure_turn_rate, self.unsure_turn_success, self.turn_gain)
            self.target_time = now
        if self.target_time is None or (now - self.target_time) * 1e-9 > self.cmd_timeout:
            self.target = (0.0, 0.0, 0.0)
        dv = self.acc_xy * self.cmd_period
        dw = self.acc_yaw * self.cmd_period
        self.out = [approach(self.out[0], self.target[0], dv),
                    approach(self.out[1], self.target[1], dv),
                    approach(self.out[2], self.target[2], dw)]
        cmd = Twist()
        cmd.linear.x, cmd.linear.y, cmd.angular.z = self.out
        self.gz_cmd_pub.publish(cmd)

    # ------------------------------------------------------------- odometry
    def on_truth(self, msg):
        pose = msg.pose.pose
        truth = (pose.position.x, pose.position.y, yaw_of(pose.orientation))
        if self.prev_truth is None:
            self.prev_truth = truth
            return
        px, py, pyaw = self.prev_truth
        self.prev_truth = truth
        # True motion since the last message, in the robot's own frame.
        c, s = math.cos(pyaw), math.sin(pyaw)
        wx, wy = truth[0] - px, truth[1] - py
        dx, dy = c * wx + s * wy, -s * wx + c * wy
        dyaw = wrap(truth[2] - pyaw)
        dist = math.hypot(dx, dy)
        # What Oak's step counting would report: scaled, noisy, drifting.
        nx = dx * (1 + self.scale_xy)
        ny = dy * (1 + self.scale_xy)
        if dist > 0:
            sd = self.noise_xy * math.sqrt(dist)
            nx += self.rng.normal(0, sd)
            ny += self.rng.normal(0, sd)
        nyaw = dyaw * (1 + self.scale_yaw) + self.drift_yaw * dist
        if dyaw != 0:
            nyaw += self.rng.normal(0, self.noise_yaw * math.sqrt(abs(dyaw)))
        ox, oy, oyaw = self.odom
        c, s = math.cos(oyaw), math.sin(oyaw)
        self.odom = [ox + c * nx - s * ny, oy + s * nx + c * ny, wrap(oyaw + nyaw)]
        if self.odom_true_speed:
            tw = msg.twist.twist
            self.odom_twist = (tw.linear.x, tw.linear.y, tw.angular.z)

        now = self.get_clock().now()
        if self.last_odom_pub is not None and \
                (now - self.last_odom_pub).nanoseconds < 1e9 / self.odom_rate:
            return
        self.last_odom_pub = now
        self.publish_odom(now.to_msg())

    def publish_odom(self, stamp):
        x, y, yaw = self.odom
        qz, qw = math.sin(yaw / 2), math.cos(yaw / 2)
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = self.base_height
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf.sendTransform(t)
        self.odom_started = True
        o = Odometry()
        o.header = t.header
        o.child_frame_id = 'base_link'
        o.pose.pose.position.x = x
        o.pose.pose.position.y = y
        o.pose.pose.position.z = self.base_height
        o.pose.pose.orientation.z = qz
        o.pose.pose.orientation.w = qw
        o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.angular.z = \
            self.odom_twist
        self.odom_pub.publish(o)

    # ------------------------------------------------------------- lidar
    def on_points(self, msg):
        if not self.odom_started:
            return
        offsets = {f.name: f.offset for f in msg.fields}
        if not all(k in offsets for k in 'xyz') or msg.point_step == 0:
            return
        n = msg.width * msg.height
        dtype = np.dtype({'names': ['x', 'y', 'z'], 'formats': ['<f4'] * 3,
                          'offsets': [offsets['x'], offsets['y'], offsets['z']],
                          'itemsize': msg.point_step})
        raw = np.frombuffer(bytes(msg.data), dtype=dtype, count=n)
        pts = np.stack([raw['x'], raw['y'], raw['z']], axis=1).astype(np.float64)
        pts = pts[np.isfinite(pts).all(axis=1)]
        rng = np.linalg.norm(pts, axis=1)
        pts = pts[(rng > 0.06) & (rng < self.lidar_range - 0.01)]
        # Sensor frame -> base_link (the lidar is mounted level).
        pts = pts + np.array(self.lidar_mount)
        body = (np.abs(pts[:, 0]) < self.self_box[0]) & (np.abs(pts[:, 1]) < self.self_box[1])
        pts = pts[~body]
        now_ns = self.get_clock().now().nanoseconds
        if self.blind_drawn_ns is None or (now_ns - self.blind_drawn_ns) * 1e-9 >= self.blind_hold:
            count = int(round(self.blind_fraction * self.blind_sector_count))
            self.blind = self.rng.choice(self.blind_sector_count, count, replace=False)
            self.blind_drawn_ns = now_ns
        pts = pts[~blind_points(pts, self.blind, self.blind_sector_count)]
        if self.z_noise > 0:
            pts[:, 2] += self.rng.normal(0.0, self.z_noise, len(pts))
        # base_link -> odom, with Oak's own (noisy) pose estimate.
        x, y, yaw = self.odom
        c, s = math.cos(yaw), math.sin(yaw)
        out = np.empty((len(pts), 4), dtype=np.float32)
        out[:, 0] = x + c * pts[:, 0] - s * pts[:, 1]
        out[:, 1] = y + s * pts[:, 0] + c * pts[:, 1]
        out[:, 2] = pts[:, 2] + self.base_height
        out[:, 3] = 1.0
        cloud = PointCloud2()
        cloud.header.stamp = self.get_clock().now().to_msg()
        cloud.header.frame_id = 'odom'
        cloud.height = 1
        cloud.width = len(out)
        cloud.fields = [PointField(name=k, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                        for i, k in enumerate(['x', 'y', 'z', 'intensity'])]
        cloud.is_bigendian = False
        cloud.point_step = 16
        cloud.row_step = 16 * len(out)
        cloud.is_dense = True
        cloud.data = out.tobytes()
        self.cloud_pub.publish(cloud)

    # ------------------------------------------------------------- joints
    def publish_joints(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = JOINTS
        js.position = STANDING
        self.joint_pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = SimDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
