from setuptools import find_packages, setup


package_name = 'go2_scoreboard'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
    description='Live and replayable scorecards for Oak mapping missions',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'recorder = go2_scoreboard.recorder:main',
            'replay = go2_scoreboard.replay:main',
        ],
    },
)
