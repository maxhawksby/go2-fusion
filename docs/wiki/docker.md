---
title: Docker
parent: lab-setup
code: [docker/**, requirements.txt]
---

# Docker

Docker is how every teammate runs Oak's code: one shared box holding Linux, ROS 2 Humble and all the libraries, so "it works on my machine" means it works on everyone's.

Part of: [Lab setup](lab-setup.md)

## How it works

Two words to know: an **image** is the recipe-baked box, built once; a **container** is a running copy of it.

Our image holds **only the dependencies** (ROS packages, PyTorch, Open3D, Gazebo Fortress for the [Simulator](simulator.md) and so on), not our code.
When the container starts, Docker **mounts** your repo's `src/` folder inside it at `/ros2_ws/src`, so the container sees your edits instantly.
Only `src/` is mounted: other repo folders such as `docs/`, `scripts/` and `host_tools/` are not inside the container.
Then the start-up script (`entrypoint.sh`) builds the workspace with `colcon build --symlink-install` and runs the default command, `ros2 launch go2_robot_sdk robot.launch.py` (the full robot stack).
Build output is kept in Docker volumes, so each start only rebuilds what changed.

"Symlink install" means the installed Python files are shortcuts to your source files, which is why Python edits need no rebuild.

## The edit loop

| What you changed | What to do |
| --- | --- |
| Python code | Restart the node (Ctrl+C and rerun, or `docker compose restart`). No rebuild. |
| C++ code, a new package, or a new executable in `setup.py` | In a container shell: `cbs <package>`, then restart the node |
| `package.xml`, `requirements.txt` or a Dockerfile | `docker compose up --build` (or `docker compose build`) |

Shortcuts inside the container: `cb` (build everything), `cbs <pkg> [pkg...]` (build only these), `teleop` (keyboard driving).

## Everyday commands (run from `docker/`)

```shell
docker compose up                          # start the full robot stack (needs ROBOT_IP in the lab)
docker compose run --rm unitree_ros true   # just build everything and exit: "does my change compile?"
docker compose exec unitree_ros bash       # open a shell in the running container
docker compose restart unitree_ros         # restart the whole stack
docker compose down                        # stop it
```

On Windows from PowerShell, add `-f docker-compose.windows.yml` after `docker compose`.

## The files

- `Dockerfile` - the main image (CPU-only PyTorch, to avoid a multi-GB GPU download).
- `docker-compose.yml` - for Linux and WSL: shares your display (for RViz), gamepads (`/dev/input`) and the host network.
- `docker-compose.windows.yml` - for Windows PowerShell: no display or gamepad, and publishes Foxglove on port 8765 (change with `FOXGLOVE_PORT`).
- `docker-compose.gpu.yml` + `Dockerfile.gpu` - optional, on top of the Windows file, for an NVIDIA GPU: YOLO runs on CUDA and RViz draws on the GPU.
- `mac/` - the Mac setup (see [Mac setup](mac-setup.md)).
- `entrypoint.sh`, `dev_aliases.sh` - start-up build and the shortcuts above.
- `../requirements.txt` - Python libraries. ROS libraries go in each package's `package.xml` instead.

## Connects to

- [Command cheat sheet](commands.md): copy-paste commands for RViz, the map viewers, keyboard driving, auto mapping and full rebuilds.
- [WSL and Windows](wsl.md): which compose file to use on Windows.
- [Lab checklist](lab-checklist.md): starting the container against the real Oak.
- [Go2 driver](go2-driver.md): what the container launches by default.

## How it breaks

- **Out of memory during the build** (crashed WSL at 7.4 GB): build with `MAKEFLAGS=-j4 colcon build --symlink-install --parallel-workers 2`, or raise WSL's memory.
- **`The aioice submodule is missing`**: run `git submodule update --init --recursive` on your machine.
- **Build in a strange state**: `docker compose down -v` deletes the build output; the next start rebuilds from scratch in about a minute.
- **Do not run `docker system prune`** or delete images: it forces a 15 minute rebuild.
- **A build seems stuck**: read the step name. A multi-GB `nvidia-*` download means PyTorch is pulling the GPU build; it should come from the CPU-only index.
- **A Python fix does not show up**: the installed copy can be stale. Rebuild that package with `cbs <pkg>`.
- **Full rebuild with `--no-cache`** takes 30-60+ minutes and has failed on flaky mirrors; check the image date with `docker image inspect` rather than trusting the log.

## Who's on it and what's next

TODO(owner): no single owner. Luis restored the Foxglove port; Noah moved packages under `src/`.

## The code

https://github.com/maxhawksby/go2-fusion/tree/master/docker

<!-- sources: docker/Dockerfile; docker/entrypoint.sh; docker/dev_aliases.sh; docker/docker-compose.yml; docker/docker-compose.windows.yml; docker/docker-compose.gpu.yml; docker/LAB_CHECKLIST.md "Build habits"; CONTRIBUTING.md "The edit loop"; AGENTS.md build and dependencies; docs/USEFUL_COMMANDS.md rebuilding; docs/TEST_LOG.md build row; docs/FUSION.md rows 6 and 15; git log cc38921 -->
