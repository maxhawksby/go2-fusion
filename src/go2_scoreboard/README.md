# Go2 scoreboard

Records mapping runs without controlling Oak. It measures odometry, maps,
automapper events, collisions, human touches, CPU and RAM; writes a report and
rosbag; and can regenerate the same metrics from a bag.

Build and test:

```shell
colcon build --symlink-install --packages-select go2_interfaces go2_auto_mapper go2_scoreboard go2_sim
source install/setup.bash
python3 -m pytest -q /ros2_ws/src/go2_scoreboard/test
```

Run before starting the auto mapper:

```shell
ros2 run go2_scoreboard recorder --ros-args -p source:=oak -p run_label:=lab-baseline
```

Wait for `Scoreboard waiting for exploration` before starting the auto mapper;
that message is logged only after optional rosbag startup is ready.

Replay a recorded bag:

```shell
ros2 run go2_scoreboard replay /path/to/run/bag --out /path/to/replays
```

See [the Scoreboard wiki page](../../docs/wiki/scoreboard.md) for topics,
services, output files and failure modes.
