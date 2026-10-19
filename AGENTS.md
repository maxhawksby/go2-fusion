# Agent guide

Instructions for AI coding agents (Claude Code, Codex, Cursor, ...) working in this repo. Humans: the same workflow is explained in [CONTRIBUTING.md](CONTRIBUTING.md).

This is a ROS 2 Humble workspace for a Unitree Go2 robot dog, worked on by a 5-person team on mixed machines (WSL2 and Windows). Everyone builds and runs through Docker so the environment is identical.

## Packages

All ROS packages live under `src/`.

| Package | What it is |
| --- | --- |
| `go2_robot_sdk` | The driver node (WebRTC/CycloneDDS connection to the robot) and all launch files |
| `go2_interfaces` | Custom messages |
| `keyboard_teleop` | Drive the robot from the keyboard |
| `go2_auto_mapper` | Autonomous exploration for SLAM mapping |
| `go2_map_viewer_cpp` | SFML map viewer: click to send a Nav2 goal |
| `lidar_processor`, `lidar_processor_cpp` | Point cloud processing |
| `go2_rviz_plugins` | RViz panels: map viewer, auto mapper, keyboard teleop |
| `coco_detector` | Object detection on the camera feed (MobileNet, `/detected_objects`); to be retired in favor of `object_detection` |
| `object_detection` | YOLOv8 detector with rough object positions and a 20 s memory (`/detected_objects/poses`, `/detected_objects_dict`) |
| `object_navigation` | Walks toward a detected object (`/navigate_to_object`); no obstacle avoidance |
| `go2_llm_agent` | Plain-English commands to robot actions via an LLM. Being rebuilt on Claude (plan in `docs/plans/pending/`): its pip deps are not in the image, so it builds but does not run |
| `speech_processor` | Text to speech through the robot's speaker |
| `front_leg_controller` | Leg/posture commands |
| `go2_dashboard_bridge` | Optional local HTTP adapter for the Oak Control UI; read-only unless commands are explicitly enabled |

`host_tools/` holds tools that run on the host, outside Docker (not ROS packages): `speech_to_text/vad_listen.py` sends microphone transcripts to `go2_llm_agent`. [FUSION.md](docs/FUSION.md) records what came from Riley Francis's ROS2-LLM-Nav and why.

## Build and run: always in Docker

The image holds only dependencies. The repo is mounted at `/ros2_ws/src`, and the container runs `colcon build --symlink-install` on start. Never install ROS or packages on the host.

```shell
docker info                                          # if this fails, ask the human to start Docker Desktop; never start/restart it yourself
cd docker
docker compose run --rm unitree_ros true             # build the whole workspace and exit: use this to check your change compiles
docker compose exec unitree_ros bash -ic 'cbs <pkg>' # rebuild one package in an already-running container
```

On Windows hosts, add `-f docker-compose.windows.yml` to every `docker compose` command.

- Python edits need no rebuild (symlink install), only a node restart.
- C++ changes, new packages, or new `entry_points` in `setup.py` need `cbs <pkg>`.
- Dependency changes need `docker compose build` (only the dependency layer reruns).
- `ros2 run <pkg> <executable>` runs a node; `.launch.py` files go through `ros2 launch <pkg> <file>`.

## Dependencies

- ROS/system dependencies go in the package's `package.xml` as a rosdep key, so the image, CI and every teammate get them automatically. Check the key exists (`rosdep resolve <key>` in the container, or rosdistro's `rosdep/base.yaml`); keys often differ from apt names (SFML is `sfml-dev`, not `libsfml-dev`). Never tell people to `apt-get install` something by hand.
- Python dependencies go in `requirements.txt`.

## Design rules

- **Extend before adding.** AI agents tend to add a new node or topic per feature, which sprawls the ROS graph. Before creating one, check whether an existing node should own the behavior or an existing topic already carries the data. Say why if you still add one.
  - Motion: publish `geometry_msgs/Twist` on your source's own twist_mux input; twist_mux forwards the highest-priority active one to the driver. Inputs (`src/go2_robot_sdk/config/twist_mux.yaml`, priority high to low): `/cmd_vel_joy` 10, `/cmd_vel_keyboard` 9, `/cmd_vel` 5 (Nav2), `/cmd_vel_agent` 4, `/cmd_vel_object_nav` 3. A new drive source gets its own input there rather than sharing one, or sources overwrite each other.
  - Robot API commands (sit, stand, modes): `/webrtc_req`.
  - Speech: `/tts`.
  - Sensors from the driver: `/odom`, `/imu`, `/joint_states`, `/point_cloud2`, `/scan`, `/camera/image_raw`, `/go2_states`.
- **ROS 2 Humble only.** Don't use Jazzy-only APIs.
- **Test without the robot first.** There is one robot and it accepts one connection at a time. Verify what your code publishes with `ros2 topic echo <topic>`. Never send motion commands to a live robot unless the human asks.
- Never commit secrets: `ROBOT_AES_KEY`, lab wifi or router credentials.

## Git

- Never commit to `master`. Branch from the latest master: `git fetch && git switch -c feat/<short-name> origin/master`.
- Keep branches small (one feature or fix). Before opening a PR: `git fetch && git rebase origin/master`.
- Merge through a GitHub pull request reviewed by a teammate; don't push to `master` directly.
