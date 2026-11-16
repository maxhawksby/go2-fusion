from glob import glob

from setuptools import find_packages, setup

package_name = 'go2_sim'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/worlds', glob('worlds/*')),
        ('share/' + package_name + '/models/oak', glob('models/oak/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    # Declaring pytest makes colcon run the tests with pytest; without it
    # colcon falls back to unittest and finds 0 tests.
    extras_require={
        'test': [
            'pytest',
        ],
    },
    description='Gazebo Fortress simulator for Oak (Go2) and auto mapper trial runner',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'sim_driver = go2_sim.sim_driver:main',
            'trials = go2_sim.trials:main',
            'ground_truth = go2_sim.ground_truth:main',
            'watch = go2_sim.watch:main',
            'score_decisions = go2_sim.decision_score:main',
        ],
    },
)
