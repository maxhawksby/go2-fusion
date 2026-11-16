---
title: Object detection
parent: senses
code: [src/object_detection/**, src/coco_detector/**, host_tools/yolo_server/**]
verified: 916c5a848d56
---

# Object detection

Object detection is Oak's computer vision: it looks at each camera picture, draws a box around things it recognizes (a person, a chair, a bottle), and remembers roughly where they are so other programs can ask "where is `chair_1`?".

Part of: [Senses](senses.md)

## How it works

**Seeing.** The main detector is `object_detection`. It uses YOLOv8 "nano", a small neural network (a program trained on thousands of labelled photos) that looks at one picture and returns boxes, each with a class name such as `chair` and a confidence from 0 to 1. Think of a friend glancing at a photo and saying "that's a chair, I'm 80% sure". YOLO knows the 80 everyday classes of the COCO photo set. Oak's computer has no graphics card in Docker, so YOLO runs on the CPU, at most 5 pictures a second (`max_rate_hz`) and on 4 CPU threads (`num_threads`), so it does not starve the driver, SLAM and Nav2.

**Guessing where.** Oak has one ordinary camera, not a depth camera, so it cannot measure distance directly. The node guesses instead:

- Direction comes from where the box sits left to right in the picture (it assumes the camera sees 90 degrees wide).
- Distance comes from box height: distance = 1 / (box height / picture height). A box that fills the whole picture counts as 1 m away, a box half as tall as 2 m. It is like judging how far a person is by how tall they look.

**Remembering.** Each guess goes into a small memory, a dictionary of IDs like `chair_1` and `chair_2`. A new box within about 0.5 m of a remembered object of the same class is treated as the same object and keeps its ID; otherwise it gets the next number. Anything not seen for 20 seconds is forgotten. The memory is published as JSON text, for example `{"chair_1": {"x": 2.1, "y": -0.3}}`, where x is meters ahead and y is meters to the side.

**On the map, not the robot.** Each guess is moved onto the room map with TF, using where the camera was when the picture was taken. The memory lives in the `map` frame when [SLAM](slam.md) runs, else `odom`, else `base_link`; if `map` appears later, remembered objects move over and keep their IDs. So a chair keeps its ID while Oak turns or walks, and ID numbers are never reused. Boxes YOLO is less than 50% sure of are ignored (`min_confidence`). The JSON keeps `x`/`y` (meters from Oak as it stands now, for older readers) and adds `map_x`, `map_y`, `frame`, `label` and `confidence`. Max built this; Luis merged it with the people tracking below.

**People.** Ideas taken from [SocialNav](https://github.com/ram95d/SocialNav) (MIT), a perception stack for a smart walker. Each `person` box is moved into the `odom` frame with TF, from the same camera position as the objects, so a person keeps one ID (`person_3`) while Oak or they move. From their last few positions (8 points, 0.4 s apart) the node works out how fast they are walking and draws a straight line 4.8 s ahead: "if they keep going, they will be here". People closer than 1.5 m to each other count as a group, like two people talking, so a robot should go around them, not between them. This is published on `/people` and drawn in RViz on `/people/markers`. The [auto mapper](auto-mapper.md) uses it to keep goals away from people and to let them pass. SocialNav itself predicts paths with a trained network on a GPU and measures distance with a depth camera; Oak has neither in Docker, so this version uses the straight-line guess and the box-height distance above.

**Any object by name (optional).** With YOLOE weights (`model_path:=yoloe-26s-seg.pt`, the open-vocabulary model SocialNav uses) the `classes` parameter lists what to look for in plain words, for example `person,chair,red box`, instead of YOLO's fixed 80 classes.

**The older detector.** `coco_detector` is a second, older detector that uses a different network (MobileNet, from the torchvision library). It publishes the standard ROS message `vision_msgs/Detection2DArray` on `/detected_objects`, with a default confidence cut-off of 0.5 (it was 0.9, which missed most objects) and no positions or memory. It is kept for now and is planned to be retired (see [Decisions](decisions.md), follow-up 3).

**Settings.** `object_detection` was written by Riley Francis in [ROS2-LLM-Nav](https://github.com/RileyFrancis/ROS2-LLM-Nav); its pose topic was renamed from `/detected_objects` to `/detected_objects/poses` when it came into this repo, and later replaced by the labeled markers on `/detected_objects/markers`. Its parameters:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `model_path` | `share/object_detection/models/yolov8n.pt` | YOLO weights file |
| `num_threads` | `4` | CPU threads for PyTorch. Unlimited, it takes every core and starves the driver, SLAM, Nav2 and RViz |
| `max_rate_hz` | `5.0` | Most pictures per second to run YOLO on; pictures in between are dropped. `0` runs on every picture |
| `min_confidence` | `0.5` | Boxes YOLO is less sure of are ignored. Lower finds more, but more false ones |
| `fixed_frame`, `fallback_frame` | `map`, `odom` | Where the memory lives, and the frame used while `map` is missing |
| `classes` | empty | Comma-separated names to look for; needs YOLOE weights. Empty keeps the model's own classes |
| `track_people` | `True` | Track people, forecast their walk and find groups (`/people`) |
| `inference_url` | `YOLO_URL`, else empty | Address of a YOLO server; empty runs YOLO here on the CPU |
| `inference_timeout_sec` | `1.0` | How long to wait for the server before using the CPU instead |

To change the default weights, add the file to `models/`, list it in `setup.py` `data_files`, and update the default in `detector_node.py`. It needs `ultralytics` from pip, which is in the repo's `requirements.txt`, so the Docker image has it.

`coco_detector` comes from [jfrancis71/ros2_coco_detector](https://github.com/jfrancis71/ros2_coco_detector) (demo: https://www.youtube.com/watch?v=emUs0nwwde8), chosen for its simple code and standard ROS messages. Its parameters: `publish_annotated_image` (default `True`), `device` (default `auto`, which uses a GPU if one is available and else the CPU; `cpu` or `cuda` forces one, and the node logs which it runs on), `detection_rate_hz` (default 10, the most pictures per second it runs on; 0 or less means every picture), and `detection_threshold` (default 0.5, between 0.0 and 1.0; higher rejects more detections, so raise it if you get too many false ones). It puts the class name straight into the message's `class_id` (for example `dog`) instead of an ID to look up. The first run downloads the MobileNet network (`fasterrcnn_mobilenet_v3_large_320_fpn`), then it is cached.

**YOLO on the GPU server (optional).** YOLO can run on the team's GPU server instead of the laptop CPU, the same way as the auto mapper's VLM. `host_tools/yolo_server/server.py` runs there; set `YOLO_URL` (for example `http://host.docker.internal:8200/detect` on Windows) and the node sends each camera picture, shrunk to 640 pixels wide, through an SSH tunnel and gets the boxes back. Everything else (positions, memory, people) still happens on the laptop. If the server does not answer within `inference_timeout_sec`, the node loads YOLO on the CPU and uses it for 30 s before asking the server again, so detection carries on while the server is down. If the CPU model cannot load either (for example the weights are missing), the node logs it, reports no objects and tries again 30 s later. Without a server, a CPU model that cannot load still stops the node at start-up, as before. The picture on `/detections/image_annotated` looks plainer with the server: orange boxes and labels, no YOLOE masks. The server refuses pictures over about 8.8 million pixels, more than 4 requests or 8 connections at once, and closes any connection still open 15 s after it started. **Camera pictures leave the robot** when this is on. Setup and the tunnel: `host_tools/yolo_server/README.md`.

## Connects to

- [Camera](camera.md) - both detectors read `/camera/image_raw`, the driver's camera topic.
- [Auto mapper](auto-mapper.md) - reads `/people` to keep goals out of people's way and wait for them.
- [Object navigation](object-navigation.md) - reads `/detected_objects_dict` and walks toward an object by ID.
- [LLM agent](llm-agent.md) - its `look_for_object` tool is not wired to this memory yet; the agent's README says a fix would read `/detected_objects_dict`.
- [SLAM](slam.md) and [TF and robot model](tf.md) - the memory lives on the SLAM map (`map` frame), and people in `odom`; both come from TF.
- [RViz panels](rviz-panels.md) - you can watch the boxes in RViz with an Image display.

Topics (a topic is a named channel that ROS programs publish messages to and listen on):

| Topic | Who | What |
| --- | --- | --- |
| `/camera/image_raw` | both read | Camera pictures from the driver |
| `/detections/image_annotated` | `object_detection` writes | Picture with boxes and labels drawn on; only drawn while someone watches it |
| `/detected_objects/markers` | `object_detection` writes | RViz: a sphere and an `ID (confidence)` label per remembered object |
| `/detected_objects_dict` | `object_detection` writes | The JSON memory |
| `/people` | `object_detection` writes | JSON, `odom` frame: each person's `x`, `y`, speed `vx`/`vy` (m/s), `group` (a number, or null when alone) and `path` (12 points, 0.4 s apart) |
| `/people/markers` | `object_detection` writes | RViz: orange dot per person, yellow forecast line, blue ring per group; only built while someone watches |
| `/detected_objects` | `coco_detector` writes | Standard detection message, no positions |
| `/annotated_image` | `coco_detector` writes | Picture with boxes |

## How it breaks

- **Positions are rough.** No depth camera and no use of the camera calibration: distance is the box-height guess above, so TF only moves a rough guess onto the map. A chair half out of view looks shorter, so it looks farther away.
- **Speed.** About 4.5 pictures a second on CPU, 55 to 130 ms each, measured on Oak on 2026-10-11 ([Test log](test-log.md)). Results lag the camera by about one picture.
- **YOLO server down or slow** (tunnel closed, server off, wrong address): the log says "YOLO server not answering" once, then the CPU model takes over, which takes a few seconds to load the first time. "YOLO server answering again" means it is back. A slow network adds its delay to every picture; each wait is limited to `inference_timeout_sec`. TODO(Luis): measure pictures per second and delay through the tunnel to the GPU server.
- **First run needs internet.** `ultralytics` downloads a font the first time it draws boxes on the laptop (not with the YOLO server), and `coco_detector` downloads its network.
- **People forecasts are only as good as the positions.** They use the same rough distance guess. A person partly out of the picture seems to jump away, which looks like a fast walk for a moment. The straight line ignores turns and other people.
- **No people without TF.** People need the `odom` pose from the driver; without it the node warns every 5 s ("No odom pose, people not tracked") and `/people` stays quiet. Objects still work.
- **YOLOE is untested on Oak.** TODO(Luis): confirm `yoloe-26s-seg.pt` downloads and runs in the container (it also downloads a text model for `classes`), and how many pictures a second it manages on the CPU.

## Who's on it and what's next

**Max** owns computer vision. His goal: YOLO that remembers objects in a local memory, to make mapping rooms easier.

TODO(owner): Max to add the plan, for example longer memory than 20 seconds and retiring `coco_detector`.

## The code

- YOLO detector: https://github.com/maxhawksby/go2-fusion/tree/master/src/object_detection (main file `object_detection/detector_node.py`)
- Older detector: https://github.com/maxhawksby/go2-fusion/tree/master/src/coco_detector

No launch file starts either detector; run it by hand inside the Docker container while the robot stack runs:

```shell
cd docker
docker compose exec unitree_ros bash -ic 'ros2 run object_detection detector_node'
docker compose exec unitree_ros bash -ic 'ros2 topic echo /detected_objects_dict'   # second terminal
```

Try other weights without rebuilding: add `--ros-args -p model_path:=/path/to/weights.pt`. Python edits need only a node restart, no rebuild. On a Windows host add `-f docker-compose.windows.yml` after `docker compose`.

<!-- sources: src/object_detection/README.md, src/object_detection/object_detection/detector_node.py, src/coco_detector/coco_detector/coco_detector_node.py, src/coco_detector/README.md, docs/FUSION.md (rows 1, follow-up 3), docs/TEST_LOG.md (2026-10-11), coco_detector README parameters and origin (2026-10-22 README trim), AGENTS.md, src/go2_llm_agent/README.md, src/object_navigation/README.md, origin/feat/yolo-map-memory commits b078372 2e66325 3ac8bce and its object_detection README, contract team updates week of 2026-10-18 -->
