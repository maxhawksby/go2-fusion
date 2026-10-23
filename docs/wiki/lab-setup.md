---
title: Lab setup
parent: oak
---

# Lab setup

Welcome to Lab setup: this is everything you need to get the code running on your own laptop and, later, to connect it to the real Oak in the lab.

Part of: [Oak](../../README.md)

## The big idea

All of Oak's software runs inside **Docker**. Docker packs Linux, ROS 2 Humble and every library into one box called a **container**, so every teammate gets the exact same setup whether they use Windows, a Mac or Linux.
You never install ROS on your own computer.

Your copy of the code stays on your laptop, where you edit it normally. The container sees your files live and builds them when it starts.
So you build the Docker **image** (the box's recipe) once, and after that you only restart things.

## Setup in 5 steps (no robot needed)

1. **Install Docker Desktop** and open it. Wait until it says the engine is running.
   - Windows: also install WSL2 (Linux inside Windows). See [WSL and Windows](wsl.md) for which way to run.
   - Mac: also install XQuartz; follow [Mac setup](mac-setup.md), which wraps every command in `bash docker/mac/go2.sh`.
2. **Check Docker works.** In a terminal:
   ```shell
   docker info
   ```
   Success: a long list of details, no error.
3. **Get the code** (the `--recursive` part also downloads a small library the driver needs):
   ```shell
   git clone --recursive git@github.com:maxhawksby/go2-fusion.git
   cd go2-fusion/docker
   ```
4. **Build the image once.** The first time takes about 15 minutes; later builds are near instant unless a dependency changed.
   ```shell
   docker compose build
   docker compose run --rm unitree_ros true
   ```
   The second line compiles every package and exits. Success: a colcon `Summary: ... packages finished` line and no `Failed`.
5. **Try a dry run.** Drive a pretend robot and watch the commands:
   ```shell
   docker compose run --rm --name oak-dry unitree_ros bash
   teleop
   ```
   In a second terminal: `docker exec -it oak-dry bash -ic 'ros2 topic echo /cmd_vel_keyboard'`. Hold W and numbers appear; let go and one stop follows.

On Windows from PowerShell, add `-f docker-compose.windows.yml` right after `docker compose` in every command.

You are set up. From here, the [Docker](docker.md) page explains the daily edit loop.

## The 3 most common problems

1. **`docker info` fails or hangs.** Docker Desktop is not running, or (inside WSL) its WSL integration is off. Start Docker Desktop and wait for "Engine running". In WSL, turn on Docker Desktop > Settings > Resources > WSL Integration for your Ubuntu distro. Details: [WSL and Windows](wsl.md).
2. **The build crashes or WSL freezes.** It ran out of memory: compiling many packages at once used all 7.4 GB WSL had on one laptop. Build with fewer parallel jobs inside the container: `MAKEFLAGS=-j4 colcon build --symlink-install --parallel-workers 2`, or give WSL more memory with `memory=` in your `.wslconfig` file.
3. **The container stops at once with `The aioice submodule is missing`.** You cloned without `--recursive`. Run `git submodule update --init --recursive` and start again.

## Pages, in reading order

1. [Docker](docker.md) - how the image and container work, the edit loop, and handy shortcuts. Read this after the steps above.
2. [WSL and Windows](wsl.md) - the two ways to run on Windows, and how to get RViz windows to appear.
3. [Mac setup](mac-setup.md) - running the container on a Mac, with RViz through XQuartz.
4. [Command cheat sheet](commands.md) - copy-paste commands for RViz, the map viewers, keyboard driving, auto mapping and rebuilds.
5. [Lab checklist](lab-checklist.md) - what to do before and at the lab to connect to the real Oak.
6. [Test log](test-log.md) - what we tested on Oak, what passed, and what is still open.

## Going to the lab

Connecting to Oak needs things that are **never** in this repo: the lab network, the robot's address and, on newer firmware, a secret key.
Ask a teammate for them, then follow the [Lab checklist](lab-checklist.md).

## How Lab setup connects to the rest of Oak

- Every program page (for example [Keyboard teleop](keyboard-teleop.md) or [Nav2](nav2.md)) assumes the container from this hub is running.
- The [Driver](driver.md) is what Docker starts by default: `robot.launch.py`, the full robot stack.
- The [Test log](test-log.md) feeds the open issues on the [Oak](../../README.md) front page and the [Roadmap](roadmap.md).

## Who's on it

- Noah wrote a repeatable lab-testing workflow for his Oak Control dashboard (`docs/DASHBOARD_LAB_GUIDE.md` on branch `feat/desktop-app-prototype`, not on master yet). He also moved all ROS packages into `src/` and organized the root docs.
- Adib wrote the [Mac setup](mac-setup.md) (`docker/mac/`).
- Max keeps the [test log](test-log.md).
- TODO(owner): confirm the minimum laptop RAM (the README says about 16 GB).

<!-- sources: README.md "Start here" steps; CONTRIBUTING.md one-time setup and edit loop; AGENTS.md build section; docker/entrypoint.sh (submodule message, colcon build); docker/LAB_CHECKLIST.md (~15 min rebuild); docs/RUNNING_IN_WSL.md (WSL integration); docs/TEST_LOG.md (7.4 GB OOM, -j4 / 2 workers, open issue 5); macos_setup.md; docker/mac/go2.sh; git log 7fc71a8, cc38921, f0dba33; origin/feat/desktop-app-prototype docs/DASHBOARD_LAB_GUIDE.md -->
