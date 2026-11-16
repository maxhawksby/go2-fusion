---
title: Mac setup
parent: lab-setup
---

# Mac setup

This page gets Oak's container running on a Mac, with RViz windows on your screen, using the Mac shortcut script `bash docker/mac/go2.sh`.

Part of: [Lab setup](lab-setup.md)

All Mac configuration is in `docker/mac/`. The shared Dockerfile, compose files, dependencies and robot code stay the same for everyone. Windows and WSL users: see [WSL and Windows](wsl.md).

## Three separate steps

1. **Start the container:** Docker gives you a Linux computer with ROS 2 Humble. It compiles the repository, then waits. It does not connect to Oak.
2. **Start the driver:** a ROS program connects to Oak and publishes sensor data. This needs Oak powered on, the right Wi-Fi, and its current address.
3. **Open RViz:** a viewer subscribes to that data and draws it on your screen. Opening RViz alone does not start a robot connection.

```text
Oak -> WebRTC driver in Docker -> camera / LiDAR / odometry ROS topics
                                              |
                                         RViz in Docker
                                              |
                                     VirtualGL -> XQuartz
                                              |
                                          Mac screen
```

Your Mac edits the repository. Docker mounts its `src/` folder at `/ros2_ws/src`. Build output lives in separate named Docker volumes, not in your source folder.
Stop nodes before switching branches: running Python nodes can otherwise see a mixture of old and new code.

## One-time preparation (on the Mac)

- Install Docker Desktop and XQuartz, then open both. Do not install ROS on macOS.
- In **XQuartz > Settings > Security**, enable **Allow connections from network clients**. Quit and reopen XQuartz after changing it.
- Open Terminal in this repository (not the older `go2_ros2_sdk` repository). The scripts never create branches, commit, push, merge or change Git history.
- Initialize the submodule if it is missing: `git submodule update --init --recursive`.

Commands below run in a **Mac terminal from the repository root**, unless marked **inside Docker**. `bash docker/mac/go2.sh` is a small shortcut for `docker compose -f docker/mac/compose.yaml`; it does not install software on macOS or start Docker Desktop for you.

## Build without Oak

```bash
bash docker/mac/go2.sh auth
bash docker/mac/go2.sh build
bash docker/mac/go2.sh check
bash docker/mac/go2.sh up
bash docker/mac/go2.sh logs
```

- `auth` reads the running XQuartz server's credential. It fixes the stale cookie problem without using `xhost +`. Credentials stay outside Git at `~/Library/Caches/go2-fusion/xauth/`, with private file permissions.
- `build` makes the Linux dependency image from `requirements.txt` and all ROS `package.xml` files. It can take several minutes the first time.
- `check` compiles the workspace and exits. No robot is needed.
- `up` refreshes XQuartz access, starts the regular container, builds incrementally and waits. Look for `Summary: ... packages finished` in `logs` before using it.

The Mac image uses the Mac's own CPU type: ARM64 on Apple Silicon, AMD64 on Intel. Mesa and Xvfb draw graphics on the CPU inside Linux; VirtualGL sends the visible window to XQuartz. This is not Apple GPU or CUDA acceleration.
Builds handle two packages at once, with up to two compiler jobs per package.

Check the window with `bash docker/mac/go2.sh rviz`. RViz opens with `/camera/image_raw` and `/point_cloud2` already selected, both using **Best Effort**, and `odom` as its fixed frame. Best Effort means the viewer accepts the driver's stream without asking it to resend every missed frame. A missing `odom` warning and empty displays are expected without a driver. Close the window when done.

An optional test uses synthetic images and points on an isolated ROS domain; it never connects to Oak or sends motion commands:

```bash
docker compose -f docker/mac/compose.yaml exec -T unitree_ros bash -ic \
  'python3 /ros2_ws/src/docker/mac/tests/offline_smoke.py'
```

## Daily development without Oak

```bash
bash docker/mac/go2.sh up
bash docker/mac/go2.sh shell
```

The second command opens a **Docker terminal**; its prompt looks like `root@...:/ros2_ws#`. Inside Docker, ROS is already sourced, so `ros2` and the team's `cb`/`cbs` shortcuts work.

Edit code on the Mac. Restart a Python node after a Python edit. For C++ edits, new executables or new packages, rebuild the package you changed, from the Mac:

```bash
bash docker/mac/go2.sh rebuild go2_rviz_plugins
```

After changing `requirements.txt` or a package dependency, rebuild the image and recreate the container:

```bash
bash docker/mac/go2.sh build
bash docker/mac/go2.sh up --force-recreate
```

These commands use the shared `docker/entrypoint.sh` and `docker/dev_aliases.sh`. The separate Mac Dockerfile follows the team's dependency recipe; **when that recipe changes, update the Mac recipe as well**. Its `Dockerfile.dockerignore` applies only to the Mac build, so the root `.dockerignore` needs no edits ([Docker documentation](https://docs.docker.com/build/concepts/context/#filename-and-location)).

## In the lab: connect and view sensors

Use one direct Oak connection at a time: coordinate with teammates and close other direct clients, including the Unitree phone app. Join Oak's network and confirm its address (ask a teammate; there is no automatic discovery). See the [Lab checklist](lab-checklist.md).

Open Docker Desktop and XQuartz, then on the **Mac**:

```bash
bash docker/mac/go2.sh up
bash docker/mac/go2.sh logs
bash docker/mac/go2.sh shell
```

In that **Docker terminal**, first check for an existing driver with `ros2 node list --no-daemon`. If `/go2_driver_node` is already running, use that session or stop it with Ctrl+C in its original terminal. Otherwise, start the driver:

```bash
ros2 run go2_robot_sdk go2_driver_node --ros-args \
  -p robot_ip:=<robot address> -p conn_type:=webrtc -p enable_video:=true
```

Leave that terminal running. This starts only the sensor driver: not navigation, object navigation, keyboard driving or mapping. If the firmware needs `ROBOT_AES_KEY`, supply it privately in the environment; never put it in a tracked file or a command you share.

In a **second Mac terminal**, run `bash docker/mac/go2.sh rviz`. Expect a changing camera view and cyan dots showing nearby surfaces. The robot's body model needs the URDF and `robot_state_publisher`; this first sensor viewer deliberately shows only the camera and lidar. It is not a finished map. Use **File > Save Config** to keep display changes in the RViz settings volume.

To check incoming data, open a third shell with `bash docker/mac/go2.sh shell`, then **inside Docker**:

```bash
timeout 15s ros2 topic echo /camera/image_raw --field header --once --qos-reliability best_effort
timeout 15s ros2 topic echo /odom --field header --once
timeout 15s ros2 topic echo /point_cloud2 --field header --once --qos-reliability best_effort
```

A timestamp means a message arrived. No output after 15 seconds means nothing arrived during that check. A publisher being listed does not by itself prove it is sending. A colorspace warning means video conversion is using the CPU; it is not a successful lidar or odometry check.

The full `robot.launch.py` also starts mapping, navigation and RViz panels; use it only when you want those. On the Mac, turn off its built-in RViz (`rviz2:=false`) and open the GUI through `mac-gui`; do not run it alongside the standalone driver.

## Stop

Stop the driver with Ctrl+C, close RViz, then on the Mac: `bash docker/mac/go2.sh down`.
This removes the container and network but keeps the build, install, log and RViz volumes. Do not add `-v` unless you mean to delete saved data. The `go2-fusion-mac` project is separate from the old `go2-mac` workspace.

## Connects to

- [Docker](docker.md): the shared image and edit loop the Mac setup follows.
- [Lab checklist](lab-checklist.md): getting connected in the lab.
- [Command cheat sheet](commands.md): commands to run inside the container.

## How it breaks

- **Invalid MIT-MAGIC-COOKIE or cannot open display:** open XQuartz and run `bash docker/mac/go2.sh auth`, then retry RViz. Restarting a terminal is not the fix. The helper finds the active XQuartz display, including `:1`.
- **Credential mount missing:** run `auth` before `check` or `up`. If you deleted and recreated the credential folder while the container was running, run `bash docker/mac/go2.sh up --force-recreate`.
- **Image display says empty topic:** choose `/camera/image_raw` and Best Effort, or reopen the provided configuration. Saved settings can override the defaults.
- **Camera works but no dots:** check `/point_cloud2` with the command above. If data arrives, check RViz's PointCloud2 status and the `odom` fixed frame. If not, check the driver session and Oak's lidar.
- **Slow view:** close duplicate RViz windows. The first viewer uses 15 FPS, point rendering and queues of depth one to limit backlog. Add heavier nodes one at a time to see what uses the CPU.
- **LLM agent import errors:** expected on `master`; its libraries are not in the image (see [LLM agent](llm-agent.md)). Finishing the Mac setup does not make it ready.
- On an 8 GB Mac, the Ollama agent's heading checks sometimes timed out while Qwen was loaded (see [LLM agent](llm-agent.md)).

Mac credential tests, which use throwaway fake credentials: `python3 -m unittest discover -s docker/mac/tests -v`.

## Who's on it and what's next

Adib wrote the Mac setup (`docker/mac/`).

<!-- sources: macos_setup.md (folded in here and deleted 2026-10-22; previous lab IP deliberately left out, and its "Current Branch is feat/macos_setup" note dropped as stale); docker/mac/; docs/wiki/lab-setup.md (Adib wrote it); docs/wiki/llm-agent.md (8 GB Mac note) -->
