# object_detection

Finds objects in the robot's camera feed with YOLOv8 (the small "nano" model) and keeps a
short memory of where they are on the map, so other nodes can ask "where is `chair_1`?".

Written by Riley Francis in [ROS2-LLM-Nav](https://github.com/RileyFrancis/ROS2-LLM-Nav).
See [FUSION.md](../FUSION.md) for what changed when it was brought into this repo.

## Run

Inside the container, with the robot stack (and so the camera) running:

```shell
ros2 run object_detection detector_node
ros2 topic echo /detected_objects_dict      # in another terminal
```

You should see JSON like this (one line; spread out here):

```json
{"chair_1": {"x": 2.1, "y": -0.3,
             "map_x": 4.52, "map_y": 1.07, "frame": "map",
             "label": "chair", "confidence": 0.81}}
```

| Field | Meaning |
| --- | --- |
| `x`, `y` | Meters from the robot as it stands now (`base_link`: x forward, y left) |
| `map_x`, `map_y` | Where the object is remembered, meters in `frame` |
| `frame` | `map`, `odom` or `base_link`; see [How it works](#how-it-works) |
| `label` | YOLO class, such as `chair` or `dining table` |
| `confidence` | YOLO's confidence (0 to 1) the last time it saw the object |

Only `x` and `y` existed at first; the other fields were added later, so readers that only
use `x` and `y` (`object_navigation`, the dashboard bridge) keep working.

To see the objects in RViz, add a MarkerArray display on `/detected_objects/markers`.

## Topics

| Direction | Topic | Type | What |
| --- | --- | --- | --- |
| Subscribes | `/camera/image_raw` | `sensor_msgs/Image` | Camera frames from the driver |
| Subscribes | `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | Where the camera is on the map |
| Publishes | `/detections/image_annotated` | `sensor_msgs/Image` | Camera frame with boxes and labels drawn on it |
| Publishes | `/detected_objects/markers` | `visualization_msgs/MarkerArray` | The memory for RViz: a sphere and an `ID (confidence)` label per object |
| Publishes | `/detected_objects_dict` | `std_msgs/String` | JSON memory: object ID to rough position (fields above) |

`/detected_objects/markers` replaced `/detected_objects/poses` (`geometry_msgs/PoseStamped`),
which carried no label, so nothing could tell what each pose was. Programs should read
`/detected_objects_dict`; the markers are for people looking at RViz. (In Riley's repo the
pose topic was `/detected_objects`; `coco_detector` publishes a different message type
there, so YOLO stays off it.)

## How it works

1. Runs YOLO on camera frames (at most `max_rate_hz`) and drops boxes below
   `min_confidence`.
2. Turns each box into a rough position from the camera: the angle comes from where the box
   sits left to right in the image (assuming a 90 degree field of view), and the distance is
   `1 / (box height / image height)`, so bigger boxes count as closer.
3. Moves that position onto the map with TF, using where the camera was when the image was
   taken (the image's timestamp; if TF for that moment does not arrive within
   `tf_timeout_sec`, the latest TF). It measures from the image's own frame
   (`front_camera`) when TF knows it, else from `base_link`.
4. A new detection within 0.5 m (on the map) of a remembered object of the same class
   reuses its ID; otherwise it gets the next ID (`chair_1`, `chair_2`, ...). Numbers are
   never reused, so `chair_2` stays `chair_2` even after `chair_1` is forgotten. Because
   the memory is on the map, a chair keeps its ID while the robot turns or walks.
5. Objects not seen for 20 seconds are forgotten.
6. Each published `x`/`y` is the remembered map position turned back into "from the robot
   now" with the latest TF, so it stays right while the robot moves.

Which frame the memory uses: `fixed_frame` (`map`) when SLAM or localization publishes it,
else `fallback_frame` (`odom`, always there while the driver runs, but it drifts), else, with
no TF at all, `base_link` (the old robot-relative behavior). The node logs once when it
picks or changes the frame. On a change (say SLAM starts and `map` appears) it moves the
remembered positions into the new frame, so objects keep their IDs; only if TF cannot link
the two frames does it forget them. The dict's `frame` field says which one is in use.

## Parameters

| Parameter | Default | Meaning |
| --- | --- | --- |
| `model_path` | `share/object_detection/models/yolov8n.pt` | YOLO weights file |
| `num_threads` | `4` | CPU threads for PyTorch. Unlimited, it takes every core and starves the driver, SLAM, Nav2 and RViz |
| `max_rate_hz` | `5.0` | Most frames per second to run YOLO on; frames in between are dropped. `0` runs on every frame |
| `min_confidence` | `0.5` | Boxes YOLO is less sure of are ignored (not remembered, not drawn). YOLO's own default, 0.25, flickers through wrong boxes on a live feed, and each one would become an ID the navigator or LLM agent might act on. Lower it to find more, raise it for fewer false alarms |
| `fixed_frame` | `map` | Frame the memory lives in |
| `fallback_frame` | `odom` | Used while `fixed_frame` has no TF (no SLAM running) |
| `robot_frame` | `base_link` | The robot body. Published `x`/`y` are relative to it; also measured from it when the camera frame has no TF |
| `tf_timeout_sec` | `0.1` | How long to wait for the TF at the image's timestamp before using the latest |

Try other weights without rebuilding:
`ros2 run object_detection detector_node --ros-args -p model_path:=/path/to/weights.pt`.
To change the default, add the file to `models/`, list it in `setup.py` `data_files`, and
update the default in `detector_node.py`.

## Limits

- Positions are very rough: no depth camera and no camera calibration, so the distance
  comes from box size alone. TF places that rough guess on the map correctly, but a bad
  distance guess can still split one object into two IDs (more than 0.5 m apart), for
  example when it is partly out of view.
- On `odom` (no SLAM running) positions slowly drift as odometry does.
- The dict is published once per processed camera frame, so the robot-relative `x`/`y`
  refresh at most `max_rate_hz` times a second.
- On CPU, detections run at most `max_rate_hz` and lag the camera by about one inference
  (50 to 100 ms). The annotated image is only drawn while something subscribes to it.
- `ultralytics` downloads a font file the first time it draws boxes, so the container
  needs internet on first run.
- Needs `ultralytics` from pip; it is in the repo's `requirements.txt`, so the Docker image
  has it.
