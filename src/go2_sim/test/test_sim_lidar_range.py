import ast
from pathlib import Path

import yaml

SRC = Path(__file__).resolve().parents[2]
LAUNCH = SRC / 'go2_sim' / 'launch' / 'sim.launch.py'
SIM_YAML = SRC / 'go2_sim' / 'config' / 'sim.yaml'
SLAM_YAML = SRC / 'go2_robot_sdk' / 'config' / 'mapper_params_online_async.yaml'


def launch_lidar_range():
    tree = ast.parse(LAUNCH.read_text())
    for node in ast.walk(tree):
        if (isinstance(node, ast.Tuple) and node.elts
                and isinstance(node.elts[0], ast.Constant)
                and node.elts[0].value == 'lidar_range'):
            return float(ast.literal_eval(node.elts[1]))
    raise AssertionError(f'no lidar_range launch argument in {LAUNCH}')


def yaml_lidar_range():
    params = yaml.safe_load(SIM_YAML.read_text())
    return float(params['go2_sim_driver']['ros__parameters']['lidar_range'])


def slam_max_laser_range():
    params = yaml.safe_load(SLAM_YAML.read_text())
    return float(params['slam_toolbox']['ros__parameters']['max_laser_range'])


def test_launch_and_yaml_agree_on_lidar_range():
    launch, config = launch_lidar_range(), yaml_lidar_range()
    assert launch == config, (
        f'sim.launch.py lidar_range {launch} m != sim.yaml lidar_range {config} m')


def test_sim_lidar_reaches_slam_max_laser_range():
    sim, slam = launch_lidar_range(), slam_max_laser_range()
    assert sim >= slam, f'sim lidar {sim} m < SLAM max_laser_range {slam} m'
