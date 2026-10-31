# go2_map_viewer_julia

Read-only Julia (GLMakie) dashboard of the SLAM map and the robot's position, talking to ROS 2 through `rosbridge_server`; not a colcon package.

## Run

Needs the robot stack running (SLAM on) and a display: the Linux compose file or WSLg, not `docker-compose.windows.yml`. The first launch compiles GLMakie for several minutes. From `docker/`, start rosbridge, then the viewer in a second terminal:

```shell
docker compose exec unitree_ros bash -ic \
  "ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
docker compose exec unitree_ros bash -ic \
  "julia --threads=auto --project=/opt/go2_map_viewer_julia /ros2_ws/src/go2_map_viewer_julia/map_viewer.jl"
```

Editing `map_viewer.jl` needs no rebuild. Adding a package:

```shell
cd go2_map_viewer_julia
julia --project=. -e 'import Pkg; Pkg.add("SomePackage")'
```

Then commit `Project.toml` and `Manifest.toml` and run `docker compose build`.

Full explanation: [Map viewers wiki page](../docs/wiki/map-viewers.md)
