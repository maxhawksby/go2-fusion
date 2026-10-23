---
title: Lab checklist
parent: lab-setup
---

# Lab checklist

This is the step-by-step list for connecting your laptop to the real Oak in the lab, in the order that avoids the usual headaches: network first, phone app closed, then start the container.

Part of: [Lab setup](lab-setup.md)

## Before you go

1. **Finish [Lab setup](lab-setup.md) at home.** The image should already be built; building in the lab wastes robot time.
2. **Ask a teammate for the access details:** the lab network, the robot's address, door codes, and the robot's AES key if needed. They are never in this repo, and must never be committed.
3. **Check the firmware version** in the Unitree app (Device > Data). If it is **1.1.15 or newer**, you need the AES key. See "Go2 firmware >= 1.1.15" in the [upstream README](https://github.com/abizovnuralem/go2_ros2_sdk#go2-firmware--1115-webrtc-local-connection).
4. **Tell the team** you plan to connect. Oak takes one connection at a time.
5. **Do not run `docker system prune`** or delete images; it forces a 15 minute rebuild.

## At the lab

1. **Network first.** Join the robot's network, then check you can reach the dog:
   ```shell
   ping <robot address>
   ```
   If the ping fails, fix the network before anything else. Docker cannot fix a network problem.
2. **Close the Unitree phone app**, and make sure no teammate's laptop is connected.
3. **Start the stack** from the repo's `docker/` folder:
   ```shell
   ROBOT_IP=<robot address> CONN_TYPE=webrtc docker compose up --build
   ```
   With firmware 1.1.15+, put `ROBOT_AES_KEY=<key> ` in front. On Windows PowerShell use the Windows compose file and `$env:` variables (see [WSL and Windows](wsl.md)).
4. **Wait for success:** `Robot 0 validated and ready` in the logs, then RViz opens with lidar dots and the camera image.
5. **Before moving the robot,** make sure someone is watching Oak and the space around it is clear.

## Build habits

- **Always launch with `docker compose up --build`.** The image holds only the dependencies, so the build is a no-op unless one changed. Code changes never need it: `src/` is mounted live (see [Docker](docker.md)).
- **If a build looks stuck, read the step name in the output** to see what it is downloading. A multi-GB `nvidia-*` download means PyTorch is pulling the GPU build again; the Dockerfile should install it from the CPU-only index.

## Reading the result

| What you see | What it means |
| --- | --- |
| RViz opens, lidar and camera data appear | Connected |
| `Connection error when requesting .../con_notify` | Cannot reach the dog: check the network (step 1) |
| Same error, but the ping works | The dog was not ready yet. The driver retries on its own, waiting 5 s, then 10 s, doubling up to 60 s between tries; or run `docker compose restart unitree_ros` |
| RSA or key error during the handshake | Firmware 1.1.15+: needs `ROBOT_AES_KEY` |
| Connects, then drops or refuses | The phone app or another laptop is still connected (step 2) |
| Camera works but lidar and odometry go silent | A data stall; the driver reconnects after 5 s. If not, `docker compose up -d --force-recreate` |

## Connects to

- [Robot link](robot-link.md): what the handshake and key actually do.
- [Test log](test-log.md): write down what you tested and how it went.
- [Docker](docker.md): the commands used above.

## How it breaks

The three most common lab problems, from the records:

1. **Not on the robot's network** (the `con_notify` error). Always ping first.
2. **Phone app still connected**, so the dog refuses or drops you.
3. **Missing AES key** on newer firmware (RSA or key error).

## Who's on it and what's next

Noah documented a repeatable lab-testing workflow for his dashboard (`docs/DASHBOARD_LAB_GUIDE.md` on branch `feat/desktop-app-prototype`). TODO(owner): merge the useful parts into this checklist once that branch lands.

<!-- sources: docker/LAB_CHECKLIST.md (folded in here and deleted 2026-10-22); docs/WINDOWS_SETUP.md (lab details: ask the lab, never commit); docs/FUSION.md row 14; README.md step 3; src/go2_robot_sdk/go2_robot_sdk/main.py (retry with backoff); infrastructure/webrtc/connection_health.py (5 s stall); docs/TEST_LOG.md data stall note; origin/feat/desktop-app-prototype docs/DASHBOARD_LAB_GUIDE.md. Lab network name and robot IP deliberately omitted. -->
