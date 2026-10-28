---
title: Nav2
parent: maps-nav
code: [src/go2_robot_sdk/config/nav2_params.yaml, src/go2_robot_sdk/launch/navigation.launch.py]
---

# Nav2

Nav2 is Oak's route planner and driver: you give it a spot on the map, and it works out a safe path there and steers Oak along it, slowing or stopping for obstacles.

Part of: [Maps and navigation](maps-nav.md)

## How it works

Nav2 (Navigation 2) is the standard ROS navigation system. Oak uses it mostly as-is, with its own settings in `nav2_params.yaml`. A goal arrives on the `/navigate_to_pose` action (an "action" is a request that takes a while and reports progress, like ordering food and getting updates). Then several parts work together:

- **Costmaps.** A costmap is a copy of the map where every 5 cm square gets a "cost": 0 for open floor, highest for a wall, and a cushion of rising cost around each wall (0.55 m, the `inflation_radius`). Think of the area around furniture painted in "don't go here" shades, darker the closer you get. There are two: a **global costmap** of the whole room on the `map` frame, for planning, and a **local costmap**, a 20 x 20 m window around the robot on the `odom` frame, for steering. Both mark and clear obstacles from the live lidar `/scan`.
- **Planner.** The Smac Hybrid planner finds a path through the global costmap that a robot with Oak's footprint (0.81 m long, 0.40 m wide) can follow. It will not plan through unknown space.
- **Controller.** The DWB controller runs 10 times a second, tries out small moves and picks the one that follows the path best without hitting anything, then publishes a speed command on `/cmd_vel`. Top speed is 0.5 m/s forward and 1.0 rad/s turning.
- **Behaviors.** Spin, back up, drive on heading and wait, used for recoveries. The [Auto mapper](auto-mapper.md) uses Spin for its 360 degree scans.

Nav2's `/cmd_vel` goes into [twist_mux](twist-mux.md) at priority 5: the gamepad (10) and keyboard (9) override it, and the auto mapper's recovery moves (6) beat it.

**Two ways to start it.** `robot.launch.py` (the normal stack) starts Nav2 alongside [SLAM](slam.md), which tells Nav2 where the robot is. `navigation.launch.py` instead loads a saved map (`map:=/path/to/map.yaml` or `MAP_FILE`) and uses AMCL, which finds the robot on a map it already has rather than drawing a new one.

## Connects to

- [SLAM](slam.md) - provides `/map` and the `map` frame.
- [Lidar](lidar.md) - `/scan` feeds both costmaps.
- [twist_mux](twist-mux.md) - receives Nav2's `/cmd_vel`.
- [Auto mapper](auto-mapper.md), [Map viewers](map-viewers.md) and the [RViz panels](rviz-panels.md) "Nav2 Goal" tool - all send goals to `/navigate_to_pose`.

## How it breaks

From the repo's records and Luis's comments in `nav2_params.yaml`:

- **"Starting point in lethal space"** on 2026-10-11: the lidar marked Oak's own body as an obstacle. The 0.1 m minimum range should fix it; retest pending ([Test log](test-log.md)).
- **Floor seen as a wall.** With the 3D cloud as a source, drifting floor points filled the costmap with "lethal" cells, so the robot stopped. Now both costmaps use `/scan` only.
- **Walking into walls.** At a 3 Hz controller the robot covered about 0.2 m between corrections and drifted into walls (now 10 Hz). With `max_vel_x` 3.0 but the smoother capping at 0.5, the robot did not follow its own plan (now both 0.5). A collision check that looked only at the robot's center let the 0.36 m nose touch walls (now checks the full footprint).
- **Paths hugging walls** at a 0.25 m cushion (now 0.55 m).
- **Global costmap blind to live obstacles**: a relative topic name pointed at a topic nobody published (now absolute `/scan`).
- **Not driving at all** under `webrtc_web.launch.py`, which skips twist_mux ([Decisions](decisions.md), follow-up 6).
- **Auto mapper waiting forever for a costmap.** Nav2 used to re-send the full local costmap only when its rolling window moved, so after Oak stopped or spun in place the [auto mapper](auto-mapper.md) never got a fresh one and stalled ("timed out waiting for fresh costmap"). Found in the [Simulator](simulator.md); `always_send_full_costmap: True` on the local costmap fixes it. Not yet confirmed on Oak.
- **Blind outside the scan's height band.** `/scan` keeps only points from 0.1 m below to 0.5 m above the body center (`robot.launch.py`), so Nav2 cannot see anything lower or higher than that.

## Who's on it and what's next

Luis tuned Nav2 in October so auto-mapping explores reliably. Next: full exploration tests on Oak, and a retest of Nav2 goals and MapViewer click-to-walk on the new code ([Test log](test-log.md) open issue 1). TODO(owner): owner for Nav2 beyond the auto mapper.

## The code

- Settings: https://github.com/maxhawksby/go2-fusion/tree/master/src/go2_robot_sdk/config (`nav2_params.yaml`)
- Launch: https://github.com/maxhawksby/go2-fusion/tree/master/src/go2_robot_sdk/launch (`robot.launch.py`, `navigation.launch.py`)

Nav2 starts with the normal stack (`nav2` defaults to true). In RViz, use the "Nav2 Goal" tool to click a goal. To stop a goal from a terminal inside Docker:

```shell
cd docker
docker compose exec unitree_ros bash -ic 'ros2 action list'   # should show /navigate_to_pose
docker compose exec unitree_ros bash -ic 'ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal "{}"'
```

Only send goals with a person watching the robot.

<!-- sources: src/go2_robot_sdk/config/nav2_params.yaml (comments, costmaps, DWB, SmacPlannerHybrid, velocity_smoother, behavior_server), launch/robot.launch.py, launch/navigation.launch.py (AMCL, map arg), config/twist_mux.yaml, docs/TEST_LOG.md, docs/FUSION.md, src/go2_auto_mapper/README.md, contract team updates (Luis) -->
