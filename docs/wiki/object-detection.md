---
title: Object detection
parent: senses
code: [src/object_detection/**, src/coco_detector/**]
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

**Important today (master):** those positions are relative to the robot's body (`base_link`), not to the room. When Oak turns or walks, the same chair's x and y change, so it can get a second ID. Nothing is placed on the [SLAM](slam.md) map yet. That is exactly what Max's in-progress work fixes (see "What's next" below).

**The older detector.** `coco_detector` is a second, older detector that uses a different network (MobileNet, from the torchvision library). It publishes the standard ROS message `vision_msgs/Detection2DArray` on `/detected_objects`, with a default confidence cut-off of 0.9 and no positions or memory. It is kept for now and is planned to be retired (see [Decisions](decisions.md), follow-up 3).

**Settings.** `object_detection` was written by Riley Francis in [ROS2-LLM-Nav](https://github.com/RileyFrancis/ROS2-LLM-Nav); its pose topic was renamed from `/detected_objects` to `/detected_objects/poses` when it came into this repo. Its parameters:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `model_path` | `share/object_detection/models/yolov8n.pt` | YOLO weights file |
| `num_threads` | `4` | CPU threads for PyTorch. Unlimited, it takes every core and starves the driver, SLAM, Nav2 and RViz |
| `max_rate_hz` | `5.0` | Most pictures per second to run YOLO on; pictures in between are dropped. `0` runs on every picture |

To change the default weights, add the file to `models/`, list it in `setup.py` `data_files`, and update the default in `detector_node.py`. It needs `ultralytics` from pip, which is in the repo's `requirements.txt`, so the Docker image has it.

`coco_detector` comes from [jfrancis71/ros2_coco_detector](https://github.com/jfrancis71/ros2_coco_detector) (demo: https://www.youtube.com/watch?v=emUs0nwwde8), chosen for its simple code and standard ROS messages. Its parameters: `publish_annotated_image` (default `True`), `device` (default `cpu`; `cuda` uses a GPU if there is one), and `detection_threshold` (default 0.9, between 0.0 and 1.0; higher rejects more detections, so raise it if you get too many false ones). It puts the class name straight into the message's `class_id` (for example `dog`) instead of an ID to look up. The first run downloads the MobileNet network (`fasterrcnn_mobilenet_v3_large_320_fpn`), then it is cached.

## Connects to

- [Camera](camera.md) - both detectors read `/camera/image_raw`, the driver's camera topic.
- [Object navigation](object-navigation.md) - reads `/detected_objects_dict` and walks toward an object by ID.
- [LLM agent](llm-agent.md) - its `look_for_object` tool is not wired to this memory yet; the agent's README says a fix would read `/detected_objects_dict`.
- [SLAM](slam.md) and [TF and robot model](tf.md) - needed for the map-based memory in progress.
- [RViz panels](rviz-panels.md) - you can watch the boxes in RViz with an Image display.

Topics (a topic is a named channel that ROS programs publish messages to and listen on):

| Topic | Who | What |
| --- | --- | --- |
| `/camera/image_raw` | both read | Camera pictures from the driver |
| `/detections/image_annotated` | `object_detection` writes | Picture with boxes and labels drawn on; only drawn while someone watches it |
| `/detected_objects/poses` | `object_detection` writes | Rough position of each box, `base_link` frame, no label |
| `/detected_objects_dict` | `object_detection` writes | The JSON memory |
| `/detected_objects` | `coco_detector` writes | Standard detection message, no positions |
| `/annotated_image` | `coco_detector` writes | Picture with boxes |

## How it breaks

- **Positions are rough.** No depth camera, no use of the camera calibration, no TF lookup (object_detection README, "Limits"). A chair half out of view looks shorter, so it looks farther away.
- **No confidence filter on master.** It trusts every YOLO box, even weak ones. In the 2026-10-11 lab test it reported a chair at confidence 0.34 ([Test log](test-log.md)).
- **Left and right are flipped on master.** Objects right of the picture's center get a positive y (left, in ROS's convention), so object navigation turns the wrong way. Found and fixed on the in-progress branch (commit `b078372`).
- **ID mix-ups on master.** Matching checks whether an ID starts with the class name, so `car` can match `carrot_1`, and after an object expires a new one can reuse a live number (commit `b078372`).
- **Speed.** About 4.5 pictures a second on CPU, 55 to 130 ms each, measured on Oak on 2026-10-11 ([Test log](test-log.md)). Results lag the camera by about one picture.
- **First run needs internet.** `ultralytics` downloads a font the first time it draws boxes, and `coco_detector` downloads its network.

## Who's on it and what's next

**Max** owns computer vision. His goal: YOLO that remembers objects in a local memory, to make mapping rooms easier.

**In progress, not merged** (branch `feat/yolo-map-memory`, 3 commits on 2026-10-14):

- Each detection is moved onto the room map with TF, using where the camera was when the picture was taken. The memory lives in the `map` frame when SLAM runs, else `odom`, else `base_link`; if `map` appears later, remembered objects are moved over and keep their IDs.
- A chair keeps its ID while Oak turns or walks, and ID numbers are never reused.
- New `min_confidence` parameter, default 0.5.
- The JSON keeps `x`/`y` (robot-relative, for existing readers) and adds `map_x`, `map_y`, `frame`, `label` and `confidence`.
- `/detected_objects/poses` is replaced by `/detected_objects/markers`, labelled spheres for RViz.

TODO(owner): Max to add the plan after the branch merges, for example longer memory than 20 seconds and retiring `coco_detector`.

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
