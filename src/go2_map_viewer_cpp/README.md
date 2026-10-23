# go2_map_viewer_cpp

SFML window showing the live SLAM map and the robot; left-click to send a Nav2 goal, `C` or `Esc` to cancel.

## Run

Needs the robot stack running (SLAM and Nav2) and a display: the Linux compose file or WSLg, not `docker-compose.windows.yml`. From `docker/`:

```shell
docker compose exec unitree_ros bash -ic "ros2 run go2_map_viewer_cpp map_viewer_node"
```

## Rebuild after a C++ change

```shell
docker compose exec unitree_ros bash -ic "cbs go2_map_viewer_cpp"
```

Full explanation: [Map viewers wiki page](../../docs/wiki/map-viewers.md)
