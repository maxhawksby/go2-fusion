# object_detection

Finds objects in the robot's camera feed with YOLOv8 nano and keeps a 20 second memory of roughly where they are.

## Run

Inside the container, with the robot stack (and so the camera) running:

```shell
ros2 run object_detection detector_node
ros2 topic echo /detected_objects_dict      # in another terminal
```

You should see JSON like `{"chair_1": {"x": 2.1, "y": -0.3}}`. Try other weights without rebuilding:
`ros2 run object_detection detector_node --ros-args -p model_path:=/path/to/weights.pt`.

Full explanation: [Object detection wiki page](../../docs/wiki/object-detection.md)
