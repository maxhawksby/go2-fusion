# macOS setup

Use this guide on a Mac. Windows and WSL users keep using `WINDOWS_SETUP.md`
and `RUNNING_IN_WSL.md`. All Mac configuration is in `docker/mac/`; the existing
Dockerfile, Compose files, dependencies and robot code remain shared and unchanged.

## Understand the three separate steps

1. **Start the container:** Docker gives you a Linux computer with ROS 2 Humble.
   It compiles the repository, then waits. It does not connect to Oak.
2. **Start the driver:** a ROS program connects to Oak and publishes sensor data.
   This needs Oak powered on, the right Wi-Fi, and its current IP address.
3. **Open RViz:** a viewer subscribes to that data and draws it on your screen.
   Opening RViz alone does not start a robot connection.

```text
Oak -> WebRTC driver in Docker -> camera / LiDAR / odometry ROS topics
                                              |
                                         RViz in Docker
                                              |
                                     VirtualGL -> XQuartz
                                              |
                                          Mac screen
```

Your Mac edits the repository. Docker sees that same folder at `/ros2_ws/src`.
Build output lives in separate named Docker volumes, not in your source folder.
Stop nodes before switching branches: running Python nodes can otherwise see a
mixture of old and new code.

## One-time preparation (on the Mac)

- Install Docker Desktop and XQuartz, then open both. Do not install ROS on macOS.
- In **XQuartz > Settings > Security**, enable **Allow connections from network
  clients**. Quit and reopen XQuartz after changing it.
- Open Terminal in this repository, not the previous `go2_ros2_sdk` repository.
  In GitHub Desktop, check that **Current Branch** is `feat/macos_setup` while
  developing this feature. These scripts never create branches, commit, push,
  merge or change Git history.
- Initialize the repository's submodule if it is missing:

  ```bash
  git submodule update --init --recursive
  ```

The commands below run in a **Mac terminal from the repository root**, unless
marked **inside Docker**. `bash docker/mac/go2.sh` is a small shortcut for
`docker compose -f docker/mac/compose.yaml`; it does not install software on
macOS or start Docker Desktop for you.

## Build without Oak

```bash
bash docker/mac/go2.sh auth
bash docker/mac/go2.sh build
bash docker/mac/go2.sh check
bash docker/mac/go2.sh up
bash docker/mac/go2.sh logs
```

- `auth` reads the running XQuartz server's credential. It fixes the stale cookie
  problem without using `xhost +`. Credentials stay outside Git at
  `~/Library/Caches/go2-fusion/xauth/`, with private file permissions.
- `build` makes the Linux dependency image from this repo's `requirements.txt`
  and all ROS `package.xml` files. It can take several minutes the first time.
- `check` compiles the workspace and exits. No robot is needed.
- `up` refreshes XQuartz access, starts the regular container, builds incrementally
  and waits. Look for `Summary: ... packages finished` in `logs` before using it.

The Mac image uses the Mac's native CPU architecture: ARM64 on Apple Silicon,
AMD64 on Intel. Mesa/Xvfb render graphics on the CPU inside Linux; VirtualGL
sends the visible window to XQuartz. This is not Apple GPU or CUDA acceleration.
Builds handle two packages at once, with up to two compiler jobs per package. Julia dependencies are installed, but
precompilation waits until its optional dashboard is first used.

Check the window:

```bash
bash docker/mac/go2.sh rviz
```

RViz opens with `/camera/image_raw` and `/point_cloud2` already selected, both
using **Best Effort**, and `odom` as its fixed frame. Best Effort means the viewer
accepts the driver's stream without asking it to resend every missed frame.
A missing `odom` warning and empty displays are expected without a driver.
Close this window when finished testing.

An optional test uses synthetic images and points on an isolated ROS domain;
it never connects to Oak or sends motion commands:

```bash
docker compose -f docker/mac/compose.yaml exec -T unitree_ros bash -ic \
  'python3 /ros2_ws/src/docker/mac/tests/offline_smoke.py'
```

## Daily development without Oak

```bash
bash docker/mac/go2.sh up
bash docker/mac/go2.sh shell
```

The second command opens a **Docker terminal**. Its prompt looks like
`root@...:/ros2_ws#`. Your Mac prompt looks different. Inside Docker, ROS is
already sourced, so `ros2` and the team's `cb`/`cbs` shortcuts work.

Edit code on the Mac. Restart a Python node after a Python edit. For C++ edits,
new executables, or new packages, rebuild the affected package on the Mac:

```bash
bash docker/mac/go2.sh rebuild go2_rviz_plugins
```

Replace that example package with the one you changed. Rebuild the image after
changing `requirements.txt` or a package dependency, then recreate the container:

```bash
bash docker/mac/go2.sh build
bash docker/mac/go2.sh up --force-recreate
```

These commands use the shared `docker/entrypoint.sh` and `docker/dev_aliases.sh`.
The separate Mac Dockerfile follows the team's dependency recipe; when that
recipe changes, update the Mac recipe as well. Its `Dockerfile.dockerignore`
applies only to the Mac build, so the root `.dockerignore` needs no edits
([Docker documentation](https://docs.docker.com/build/concepts/context/#filename-and-location)).

## In the lab: connect and view sensors

Use one direct Oak connection at a time; coordinate with teammates and close
other direct clients, including the Unitree phone app if connected. Join Oak's
network and confirm its IP. `192.168.50.150` below is the previous lab address,
not automatic discovery.

Open Docker Desktop and XQuartz, then on the **Mac**:

```bash
bash docker/mac/go2.sh up
bash docker/mac/go2.sh logs
bash docker/mac/go2.sh shell
```

In that **Docker terminal**, first check for an existing driver:

```bash
ros2 node list --no-daemon
```

If `/go2_driver_node` is already running, use that session or stop it with Ctrl+C
in its original terminal before starting another. Otherwise, start the driver:

```bash
ros2 run go2_robot_sdk go2_driver_node --ros-args \
  -p robot_ip:=192.168.50.150 -p conn_type:=webrtc -p enable_video:=true
```

Leave that terminal running. This starts the sensor driver; it does not start
navigation, object navigation, keyboard driving or mapping. If the firmware
requires `ROBOT_AES_KEY`, supply it privately in the environment; never put it
in a tracked file or a command you share.

In a **second Mac terminal**, from this repository:

```bash
bash docker/mac/go2.sh rviz
```

Expect a changing camera view and cyan dots showing nearby surfaces. The robot's
body model needs URDF and `robot_state_publisher`; the initial sensor viewer
intentionally displays the camera and LiDAR only. It is not a finished map.
Use **File > Save Config** to keep display changes in the RViz settings volume.

To check incoming data, open a third shell with `bash docker/mac/go2.sh shell`.
**Inside Docker**:

```bash
timeout 15s ros2 topic echo /camera/image_raw --field header --once --qos-reliability best_effort
timeout 15s ros2 topic echo /odom --field header --once
timeout 15s ros2 topic echo /point_cloud2 --field header --once --qos-reliability best_effort
```

A timestamp means a message arrived. No output after 15 seconds means no matching
message arrived during that check. A publisher being listed does not by itself
prove that it is sending messages. A colorspace warning means video conversion
is using the CPU; it is not a successful LiDAR or odometry check.

The normal full `robot.launch.py` also starts mapping/navigation and RViz panels.
Use it only when you intend those features. On the Mac, disable its built-in
RViz (`rviz2:=false`) and open the GUI through `mac-gui`; do not run it alongside
the standalone driver. This guide first verifies sensors independently because
some shared nodes and the LLM rebuild have their own unfinished work.

## Stop and troubleshoot

Stop the driver with Ctrl+C. Close RViz, then on the Mac:

```bash
bash docker/mac/go2.sh down
```

This removes this project's container and network while retaining build, install,
log and RViz volumes. Avoid adding `-v` unless you intend to delete saved data.
The `go2-fusion-mac` project is separate from the old `go2-mac` workspace.

- **Invalid MIT-MAGIC-COOKIE / cannot open display:** open XQuartz and run
  `bash docker/mac/go2.sh auth` on the Mac, then retry RViz. Restarting a terminal
  is not the fix. The helper finds the active XQuartz display, including `:1`.
- **Credential mount absent:** run `auth` before `check`/`up`. If you deleted and
  recreated the credential directory while the container was running, use
  `bash docker/mac/go2.sh up --force-recreate` to refresh its mount.
- **Image says empty topic:** choose `/camera/image_raw` and Best Effort, or reopen
  the provided configuration. Saved user settings can override the initial defaults.
- **Camera works but no dots:** check `/point_cloud2` with the command above.
  If data arrives, inspect RViz's PointCloud2 status and `odom` fixed frame.
  If data does not arrive, inspect the driver session and Oak's LiDAR state.
- **Slow view:** close duplicate RViz windows. The initial viewer uses 15 FPS,
  point rendering and queues of depth one to limit backlog. Add heavier nodes
  individually so you can see what consumes CPU.
- **Typed LLM agent:** follow [go2_llm_agent/README.md](go2_llm_agent/README.md). Ollama runs on your Mac; the agent runs in Docker and defaults to simulation. No OpenAI/ElevenLabs key is needed.
  Completing Mac Docker setup does not make that feature ready.

Mac credential regression tests, which use disposable fake credentials:

```bash
python3 -m unittest discover -s docker/mac/tests -v
```
