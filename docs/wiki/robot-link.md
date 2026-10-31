---
title: Robot link
parent: driver
code: [src/go2_robot_sdk/go2_robot_sdk/infrastructure/webrtc/**, src/go2_robot_sdk/external_lib/**]
---

# Robot link

The robot link is the actual wire (or radio) between our laptop and Oak: it is how the driver opens a connection to the dog and keeps it alive.

Part of: [Driver](driver.md)

## How it works

There are two ways to reach the dog, chosen with `CONN_TYPE`:

**1. WebRTC over Wi-Fi (`CONN_TYPE=webrtc`, the lab default).**
WebRTC is the same technology video-call apps use to stream video and data between two devices.
The Unitree phone app uses it too, which is why only one of you can be connected at a time.
Connecting is a handshake in a few steps:

1. The driver asks the dog for its public key at a web address on the dog (`/con_notify`).
2. It makes a fresh secret key, locks the connection offer with it, and sends it back (`/con_ing_...`).
3. The dog answers, the WebRTC connection opens, and data starts flowing: a **data channel** for odometry, IMU, joints, lidar and status, and a separate **video track** for the camera.

**Newer firmware needs an extra key.** On firmware 1.1.15 and newer the dog's first answer is itself encrypted with a per-device AES key (a 32-character hex code tied to the Unitree account the dog is registered to).
You pass it as `ROBOT_AES_KEY`. Never put it in a file in the repo; ask a teammate for it.

**Staying connected.** If the connection closes, or the data channel goes quiet for 5 seconds while video keeps coming, the driver treats it as lost and reconnects, waiting 5 s, then 10 s, up to 60 s between tries.

**Watching each stream.** One stream can also go quiet while the rest keeps flowing: after a reconnect, odometry came back but the lidar cloud or the camera sometimes did not. So the driver watches the lidar and the camera separately:

- After 10 s without lidar, it asks the dog for it again (traffic saving off, lidar switched on; neither moves the robot). Still nothing 10 s later: it treats the connection as lost and reconnects.
- After 10 s without camera frames, it asks for the video again, and keeps asking. A silent camera never forces a reconnect.
- The lidar request is now sent once the connection is validated. Sent earlier, before the data channel was open, it was silently dropped.

**2. CycloneDDS over Ethernet (`CONN_TYPE=cyclonedds`).**
DDS is the networking layer ROS 2 itself uses. With a cable plugged into the dog and your computer on the robot's subnet, ROS can discover the dog's own topics directly, with no IP or key.
In this repo that path is barely used: the driver's CycloneDDS handlers are empty placeholders, and discovery through Docker Desktop on Windows is untested.

The `external_lib/` folder holds two pieces the link needs: `aioice` (a patched networking library, pulled in as a git **submodule**, a repo inside our repo) and `libvoxel.wasm` (the decoder for the dog's compressed lidar data).

## Connects to

- [Go2 driver](go2-driver.md): owns the link and publishes what comes through it.
- [Go2 interfaces](go2-interfaces.md): `WebRtcReq` messages on `/webrtc_req` become commands sent over the data channel.
- [Lab checklist](lab-checklist.md): the steps to connect in the lab.
- [Voice](voice.md): Jeth's speaker work uses a separate WebRTC driver, so it competes for the same single connection.

## How it breaks

| What you see | What it means |
| --- | --- |
| `Connection error when requesting .../con_notify` | Your laptop cannot reach the dog. Check you are on the robot's network. |
| Same error, but the network is fine | The dog was not ready yet. Wait for a retry, or `docker compose restart unitree_ros`. |
| `RSA key format is not supported`, or a key error | Firmware 1.1.15+: set `ROBOT_AES_KEY`. |
| `aes_key must be 16 bytes (32 hex chars)` or `aes_key rejected by robot` | The key is mistyped or belongs to another dog. |
| Connects, then drops or refuses | The phone app or a teammate is still connected. |
| `No lidar from robot 0 for 10 s while other data arrives; asking for it again` | The lidar went quiet. Usually it comes back; if not, the next message is `still missing after asking again` and the driver reconnects. |
| `No camera frames from robot 0 for 10 s; asking for the video again` | The camera went quiet. Lidar and driving keep working. |
| `The aioice submodule is missing` at container start | Run `git submodule update --init --recursive` on your machine. |

## Who's on it and what's next

Luis ported the reconnect and drop detection from the team fork. An unmerged branch (`fix_hz_lidar_stream`) has a faster lidar decoder (claimed about 2 Hz to 62 Hz); the plan is to port only its decoder file if lidar rate hurts mapping. TODO(owner): an owner for the link.

## The code

https://github.com/maxhawksby/go2-fusion/tree/master/src/go2_robot_sdk/go2_robot_sdk/infrastructure/webrtc

Unit tests for the stall detector and the per-stream watch (in a container shell):

```shell
cd /ros2_ws && python3 -m pytest src/go2_robot_sdk/test/test_connection_health.py
```

<!-- sources: src/go2_robot_sdk/go2_robot_sdk/infrastructure/webrtc/go2_connection.py (handshake, data2==3, AES key errors); http_client.py (/con_notify, /con_ing_); webrtc_adapter.py is_connected; connection_health.py; main.py backoff; presentation/go2_driver_node.py (cyclonedds callbacks pass); .gitmodules; docker/entrypoint.sh; docker/LAB_CHECKLIST.md "Reading the result"; docs/WINDOWS_SETUP.md troubleshooting and Ethernet section; docs/FUSION.md row 15 and follow-up 2; team update 2026-10-18 (Jeth). TODO(owner): confirm the pytest command runs in the image. -->
