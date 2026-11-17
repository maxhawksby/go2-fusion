"""Synthetic lab runs shared by the lab_measure tests."""

import math

from go2_scoreboard.lab_measure import Command, Gyro, Pose

ODOM_HZ = 50


def burst(start_s, count, linear=0.0, angular=0.0):
    """What `ros2 topic pub --times <count> --rate 10` sends."""
    return [Command(start_s + 0.1 * i, linear, angular) for i in range(count)]


def drive(duration_s, legs):
    """Poses along +x: each leg is (start_s, end_s, speed_mps); Oak stands otherwise."""
    poses = []
    x = 0.0
    for step in range(int(duration_s * ODOM_HZ) + 1):
        t = step / ODOM_HZ
        poses.append(Pose(t, x, 0.0, 0.0))
        x += sum(speed for start, end, speed in legs if start <= t < end) / ODOM_HZ
    return poses


def turn(duration_s, legs, yaw=3.0):
    """Poses turning on the spot, yaw wrapped to +-pi as /odom reports it."""
    poses = []
    for step in range(int(duration_s * ODOM_HZ) + 1):
        t = step / ODOM_HZ
        poses.append(Pose(t, 0.0, 0.0, math.remainder(yaw, math.tau)))
        yaw += sum(rate for start, end, rate in legs if start <= t < end) / ODOM_HZ
    return poses


def gyro(duration_s, legs):
    return [Gyro(step / ODOM_HZ,
                 sum((rate for start, end, rate in legs if start <= step / ODOM_HZ < end), 0.0))
            for step in range(int(duration_s * ODOM_HZ) + 1)]


# Two stop repeats at 0.3 m/s. After the last command Oak rolls on at 0.2 m/s
# for 2.0 s (0.4 m), then for 3.0 s (0.6 m).
STOP_COMMANDS = burst(2.0, 30, linear=0.3) + burst(12.0, 30, linear=0.3)
STOP_POSES = drive(22.0, [(2.0, 4.9, 0.3), (4.9, 6.9, 0.2),
                          (12.0, 14.9, 0.3), (14.9, 17.9, 0.2)])

# One 2 s turn at 0.8 rad/s. The gyro sees it 0.5 s after the first command,
# the odometry 2.0 s after.
TURN_COMMANDS = burst(1.0, 20, angular=0.8)
TURN_GYRO = gyro(8.0, [(1.5, 3.5, 0.8)])
TURN_POSES = turn(8.0, [(3.0, 5.0, 0.8)])
