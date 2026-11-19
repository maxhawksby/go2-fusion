---
title: SLAM
parent: maps-nav
code: [src/go2_robot_sdk/config/mapper_params_online_async.yaml, src/go2_robot_sdk/launch/mapping.launch.py]
verified: 9e5421dac09c
---

# SLAM

SLAM is how Oak draws a floor plan of a room while also working out where it is standing on that plan, which every other mapping and navigation program depends on.

Part of: [Maps and navigation](maps-nav.md)

## How it works

**The idea.** SLAM stands for Simultaneous Localization And Mapping. Picture walking into a dark, unfamiliar house with a flashlight. You sketch the walls you see, and you also keep track of where you are on your own sketch by recognizing walls you already drew. Both jobs need each other: you can't draw walls in the right place without knowing where you are, and you can't know where you are without the drawing. SLAM does both at once.

**What Oak uses.** Oak runs `slam_toolbox`, a standard ROS package, in "online async" mapping mode. It reads two things:

- `/scan` - the flat ring of lidar distances (see [Lidar](lidar.md)).
- Odometry, through the `odom` -> `base_link` link in TF (see [TF and robot model](tf.md)).

Each time Oak has moved 0.2 m or turned 0.2 rad (about 11 degrees), slam_toolbox lines the new scan up against what it has drawn so far ("scan matching") and adds it. When Oak comes back to a place it has seen before, it recognizes it and corrects the whole drawing ("loop closing").

**What it produces.**

- `/map` - an occupancy grid: the room cut into 5 cm squares, each marked free, occupied (wall) or unknown. Think of graph paper with squares colored white, black or gray. Published every 1 second (`map_update_interval`).
- The `map` -> `odom` transform: the correction for odometry drift, so programs can ask where Oak is on the map.

**How it starts.** The everyday `robot.launch.py` starts slam_toolbox itself, with `respawn=True`, so the RViz AutoMapper panel's "Reset maps" button can stop it and launch brings back a fresh, empty map one second later. `mapping.launch.py` is a separate, mapping-only setup that includes slam_toolbox's own `online_async_launch.py` instead (no Nav2, no respawn). Both use `mapper_params_online_async.yaml`.

## Connects to

- [Lidar](lidar.md) - provides `/scan`.
- [TF and robot model](tf.md) - odometry in, `map` -> `odom` out.
- [Nav2](nav2.md) - plans on `/map` and uses `map` to know where the robot is.
- [Auto mapper](auto-mapper.md) - looks for unexplored edges of `/map`.
- [Map viewers](map-viewers.md) and [RViz panels](rviz-panels.md) - show `/map`.
- [Object detection](object-detection.md) - pins the objects it remembers to the `map` frame (merged in [d0d7409](https://github.com/maxhawksby/go2-fusion/commit/d0d7409b7262768ccbd527f628a5c2a7388bba39)).

## How it breaks

These come from comments Luis left in `mapper_params_online_async.yaml` in October:

- **Stale map.** At the old 5 s update interval the explorer and Nav2 worked from a map up to 5 s old. Now 1 s (SLAM uses about 4% CPU, so this is cheap).
- **Too few views.** At 0.5 m / 0.5 rad, a full 360 degree turn added only about 12 scans; at 0.2 / 0.2 it adds about 30.
- **"Transform data too old".** `/scan` arrives only about 4 times a second, so with a 0.2 s time-out the `map` -> `odom` link went stale between scans and Nav2's controller stopped. Now 0.5 s.
- **Needs a look around first.** Until slam_toolbox has placed the robot, there is no `map` frame; the auto mapper turns a full circle at start for that reason.
- **No reset service** on ROS 2 Humble's slam_toolbox, hence the stop-and-respawn trick. Under `mapping.launch.py` there is no respawn, so "Reset maps" would stop SLAM for good there (from reading the two launch files; not tested).
- `min_laser_range: 0.1` matches `robot.launch.py`'s scan, but `mapping.launch.py` builds its scan with a 0.3 m minimum.

## Who's on it and what's next

Luis tuned SLAM in October for the auto mapper (map updates every second, more scans per turn). Next: full exploration tests on Oak.

## The code

- Settings: https://github.com/maxhawksby/go2-fusion/tree/master/src/go2_robot_sdk/config (`mapper_params_online_async.yaml`)
- Launch files: https://github.com/maxhawksby/go2-fusion/tree/master/src/go2_robot_sdk/launch (`robot.launch.py`, `mapping.launch.py`)

SLAM starts with the normal stack (`slam` defaults to true). Check it inside Docker:

```shell
cd docker
docker compose up -d
docker compose exec unitree_ros bash -ic 'ros2 topic hz /map'                 # about once a second
docker compose exec unitree_ros bash -ic 'ros2 run tf2_ros tf2_echo map odom'
```

**Saving a map for `navigation.launch.py`.** While SLAM is still running, save `/map` with Nav2's map saver. It comes with `nav2_map_server`, which the driver package already depends on ([package.xml](https://github.com/maxhawksby/go2-fusion/blob/426e5314386451e7720971e3eb55b425584e859e/src/go2_robot_sdk/package.xml#L21)):

```shell
docker compose exec unitree_ros bash -ic 'mkdir -p /ros2_ws/src/maps && ros2 run nav2_map_server map_saver_cli -f /ros2_ws/src/maps/lab'
```

It writes two files: `lab.pgm` (the picture) and `lab.yaml` (its scale and where it sits). Save under `/ros2_ws/src/`, because that is the only folder the container shares with your laptop ([docker-compose.yml](https://github.com/maxhawksby/go2-fusion/blob/426e5314386451e7720971e3eb55b425584e859e/docker/docker-compose.yml#L14)). On your laptop the files appear in the repo's `src/maps/`. A map saved anywhere else is lost when the container is removed. Then load it:

```shell
ros2 launch go2_robot_sdk navigation.launch.py map:=/ros2_ws/src/maps/lab.yaml
```

`navigation.launch.py` stops at once with `Map file not found` if the path is wrong ([navigation.launch.py](https://github.com/maxhawksby/go2-fusion/blob/426e5314386451e7720971e3eb55b425584e859e/src/go2_robot_sdk/launch/navigation.launch.py#L22-L30)). These steps come from the code and the standard Nav2 tool. Nobody has tried them on Oak or in the simulator yet.

<!-- sources: src/go2_robot_sdk/config/mapper_params_online_async.yaml (comments), launch/robot.launch.py (slam_toolbox node, respawn), launch/mapping.launch.py, src/go2_rviz_plugins/src/auto_mapper_panel.cpp (Reset maps), src/go2_auto_mapper/README.md, docs/USEFUL_COMMANDS.md, git log Oct 14-19 (Luis), contract team updates; map saving: src/go2_robot_sdk/package.xml (nav2_map_server), docker/docker-compose.yml (src mount), launch/navigation.launch.py (_validate_map_arg) -->
