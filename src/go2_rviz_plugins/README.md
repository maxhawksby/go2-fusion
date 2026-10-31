# go2_rviz_plugins

Three RViz panels, loaded as plugins rather than run as separate apps:

- **MapViewer** — the live SLAM map with click-to-navigate, drawn with
  QPainter instead of through RViz's OGRE scene. That's the point: RViz's
  own `Map` display is disabled in
  [single_robot_conf.rviz](../go2_robot_sdk/config/single_robot_conf.rviz)
  because it segfaults under WSLg's Mesa/llvmpipe software renderer. A
  Qt-painted panel doesn't go through that code path at all, so it works
  where the built-in display doesn't. Functionally it's the same thing as
  [go2_map_viewer_cpp](../go2_map_viewer_cpp), just docked inside RViz
  instead of its own SFML window.
- **AutoMapper** — start/stop button and live log output for
  go2_auto_mapper's explorer (see
  [go2_auto_mapper/README.md](../go2_auto_mapper/README.md) for what that
  node actually does), plus **Scan** and **Reset maps** buttons (below).
  There's no frontier count or current-goal readout here.
- **KeyboardTeleop** — click the panel to focus it, then hold keys to drive.
  Publishes `/cmd_vel_keyboard` (the same twist_mux input as
  keyboard_teleop) straight from Qt key events; functionally the same
  as [keyboard_teleop](../keyboard_teleop), just without a terminal.

## Adding the panels in RViz

1. Build the package (see [AGENTS.md](../../AGENTS.md) — `cbs go2_rviz_plugins`
   inside the container, or it's picked up by a full workspace build).
2. In RViz: **Panels menu → Add New Panel**.
3. Pick `go2_rviz_plugins` → `MapViewer`, `AutoMapper`, and/or
   `KeyboardTeleop`.
4. Save the layout (**File → Save Config**) so they reappear next time.

## MapViewer

Click anywhere on the map to send a Nav2 goal there; **Cancel Goal** cancels
whatever's active. Closing the panel or RViz also cancels the panel's goal,
so the robot doesn't keep walking with no Cancel button left. Same goal-handle-generation and pending-cancel logic as
`go2_map_viewer_cpp`'s standalone node, so rapid re-clicks or a cancel
pressed before a goal is accepted both behave correctly instead of racing.

The panel runs its own `rclcpp::Node` (`go2_rviz_map_viewer_panel`) spun on
a background thread, separate from RViz's internal ROS plumbing. ROS
callbacks only ever touch Qt widgets through signals connected with Qt's
default (auto) connection type, which queues onto the GUI thread
automatically since the emitting thread differs from the widget's own.

## AutoMapper

**Start** launches the explorer as a child process (inheriting this
process's already-ROS-sourced environment, since RViz itself was started
through `ros2 launch`). It runs the installed executable
(`<prefix>/lib/go2_auto_mapper/explorer`) directly, not through `ros2 run`,
so the explorer is RViz's own child: **Stop** reaches it, and if RViz exits
or crashes, the explorer gets `SIGINT` and shuts down cleanly. Through
`ros2 run`, the wrapper outlived RViz and the explorer kept running. If the
executable can't be found, the panel falls back to `ros2 run` and says so in
the log. **Stop** sends it `SIGINT` — not `QProcess`'s
default `SIGTERM` — specifically so the explorer node's own
`KeyboardInterrupt`/`ExternalShutdownException` handling runs and cancels
any in-flight Nav2 goal before exiting, the same clean-shutdown path
documented in `go2_auto_mapper/README.md`. If it doesn't exit within 5s,
the panel force-kills it.

**This still moves the robot on its own** — same caution as running the
explorer from a terminal: only use it when you're physically present and
watching the robot.

Two buttons call `std_srvs/Trigger` services through the panel's own
`rclcpp::Node` (`go2_rviz_auto_mapper_panel`, spun like MapViewer's); the
reply shows in the log. A request with no reply in 5 s is dropped and the
button comes back:

- **Scan** → `/frontier_explorer/scan`: the robot turns 360 degrees (it
  moves). Needs the explorer running.
- **Reset maps** (asks to confirm) starts both maps over from where the
  robot stands. It refuses while the explorer runs: press **Stop** first,
  since the explorer's goals and frontiers belong to the old map.
  - 2D: stops `slam_toolbox` (`pkill -INT -f async_slam_toolbox_node`);
    `robot.launch.py` starts it with `respawn=True`, so it comes straight
    back with an empty `/map`. Humble's slam_toolbox has no reset service.
    Navigation pauses a few seconds while `map -> odom` is missing.
  - 3D: `/lidar_to_pointcloud/reset` clears the cloud `lidar_processor`
    accumulates (`/pointcloud/aggregated`, saved to `3d_map.ply`). It
    refills right away from `/point_cloud2`, which already carries the
    robot's own local 3D map; that part lives on the robot and isn't reset.

## KeyboardTeleop

`W`/`S` forward-back, `A`/`D` turn, `Q`/`E` strafe, `Z`/`X` speed down/up,
`Space`/`Esc` stop. The panel must have focus (click it first) to receive
key events — RViz routes keyboard input to whatever widget is focused, same
as any other application.

Unlike `keyboard_teleop.py`, which reads a raw terminal and has no way to
detect a key release (only a key-repeat timeout, `AXIS_HOLD_TIMEOUT`), Qt
delivers real press/release events, so held keys are tracked directly and
there's no watchdog timer needed. Losing focus (clicking elsewhere) clears
every held key and publishes an immediate stop — if you're mid-drive and
click into another panel, it won't keep driving with no way to let go of
the key from here anymore.

Full explanation: [RViz panels wiki page](../../docs/wiki/rviz-panels.md)
