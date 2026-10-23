# go2_rviz_plugins

Three RViz panels - MapViewer (click to send a Nav2 goal), AutoMapper (start/stop the explorer, Scan, Reset maps) and KeyboardTeleop - already in the default RViz layout.

## Build and add

C++, so rebuild after a change, then restart RViz. From `docker/`:

```shell
docker compose exec unitree_ros bash -ic 'cbs go2_rviz_plugins'
```

To add a panel by hand: RViz **Panels > Add New Panel > go2_rviz_plugins**, pick the panel, then **File > Save Config**. AutoMapper moves the robot on its own; only use it while watching Oak.

Full explanation: [RViz panels wiki page](../../docs/wiki/rviz-panels.md)
