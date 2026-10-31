# object_detection

Finds objects in the robot's camera feed with YOLOv8 (the small "nano" model) and keeps a
short memory of where they are, so other nodes can ask "where is `chair_1`?".

Written by Riley Francis in [ROS2-LLM-Nav](https://github.com/RileyFrancis/ROS2-LLM-Nav).
See [FUSION.md](../../docs/FUSION.md) for what changed when it was brought into this repo.

## Run

Inside the container, with the robot stack (and so the camera) running:

```shell
ros2 run object_detection detector_node
ros2 topic echo /detected_objects_dict      # in another terminal
```

You should see JSON like `{"chair_1": {"x": 2.1, "y": -0.3}}`.

## Topics

| Direction | Topic | Type | What |
| --- | --- | --- | --- |
| Subscribes | `/camera/image_raw` | `sensor_msgs/Image` | Camera frames from the driver |
| Publishes | `/detections/image_annotated` | `sensor_msgs/Image` | Camera frame with boxes and labels drawn on it |
| Publishes | `/detected_objects/poses` | `geometry_msgs/PoseStamped` | Rough position of each detection, frame `base_link` (no label) |
| Publishes | `/detected_objects_dict` | `std_msgs/String` | JSON memory: object ID to rough `x`/`y` in meters |

`/detected_objects/poses` was `/detected_objects` in Riley's repo. It was renamed because
`coco_detector` publishes a different message type on `/detected_objects`.

## How it works

1. Runs YOLO on every camera frame.
2. Turns each box into a rough position: the angle comes from where the box sits left to
   right in the image (assuming a 90 degree field of view), and the distance is
   `1 / (box height / image height)`, so bigger boxes count as closer.
3. A new detection within about 0.5 m of a remembered object of the same class reuses its
   ID; otherwise it gets the next ID (`chair_1`, `chair_2`, ...).
4. Objects not seen for 20 seconds are forgotten.
5. People (`track_people`, ideas from [SocialNav](https://github.com/ram95d/SocialNav)) are
   moved into `odom` with TF and tracked with steady IDs. A straight-line forecast 4.8 s ahead
   and groups of people within 1.5 m of each other go out as JSON on `/people` and as RViz
   markers on `/people/markers`. Logic in `object_detection/people.py`.

## Parameters

| Parameter | Default | Meaning |
| --- | --- | --- |
| `model_path` | `share/object_detection/models/yolov8n.pt` | YOLO weights file |
| `num_threads` | `4` | CPU threads for PyTorch. Unlimited, it takes every core and starves the driver, SLAM, Nav2 and RViz |
| `max_rate_hz` | `5.0` | Most frames per second to run YOLO on; frames in between are dropped. `0` runs on every frame |
| `confidence` | `0.15` | Minimum YOLO confidence for a detection (YOLO's own default is 0.25). Lower finds more objects but more false ones |
| `classes` | empty | Comma-separated names to look for, e.g. `person,chair,red box`; needs YOLOE weights (`model_path:=yoloe-26s-seg.pt`) |
| `track_people` | `True` | Track people and publish `/people` and `/people/markers` |

Try other weights without rebuilding:
`ros2 run object_detection detector_node --ros-args -p model_path:=/path/to/weights.pt`.
To change the default, add the file to `models/`, list it in `setup.py` `data_files`, and
update the default in `detector_node.py`.

## Limits

- Positions are very rough: no depth camera, no camera calibration, no TF lookup for objects
  (people use TF only to move into `odom`).
- On CPU, detections run at most `max_rate_hz` and lag the camera by about one inference
  (50 to 100 ms). The annotated image is only drawn while something subscribes to it.
- `ultralytics` downloads a font file the first time it draws boxes, so the container
  needs internet on first run.
- Needs `ultralytics` from pip; it is in the repo's `requirements.txt`, so the Docker image
  has it.

Full explanation: [Object detection wiki page](../../docs/wiki/object-detection.md)
