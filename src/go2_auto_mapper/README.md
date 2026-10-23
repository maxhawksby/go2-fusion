# go2_auto_mapper

Frontier-exploration mapper: reads the live SLAM map, finds unexplored edges and drives the robot there through Nav2.

**This moves the robot on its own.** Only run it when you're physically present, watching the robot, in a cleared area.

## Run

From `docker/`, with the robot stack running (SLAM and Nav2 on):

```shell
docker compose exec unitree_ros bash -ic 'ros2 run go2_auto_mapper explorer'
docker compose exec unitree_ros bash -ic 'ros2 service call /frontier_explorer/scan std_srvs/srv/Trigger'   # scan on demand
```

Ctrl+C stops it and cancels its Nav2 goal. Emergency stop from another terminal:

```shell
docker compose exec unitree_ros bash -ic 'ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal "{}"'
docker compose exec unitree_ros bash -ic 'ros2 topic pub /cmd_vel_joy geometry_msgs/msg/Twist "{}" --once'
```

## Test (no robot needed)

```shell
docker compose run --rm unitree_ros bash -c 'python3 -m pytest /ros2_ws/src/go2_auto_mapper/test'
```

Full explanation, including every parameter: [Auto mapper wiki page](../../docs/wiki/auto-mapper.md)
