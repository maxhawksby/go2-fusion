# coco_detector

Older object detector: PyTorch torchvision MobileNet on `/camera/image_raw`, publishing `vision_msgs/Detection2DArray` on `/detected_objects` (to be retired in favor of `object_detection`).

## Run

Inside the container, with the robot stack (and so the camera) running:

```shell
ros2 run coco_detector coco_detector_node
ros2 topic echo /detected_objects                                        # in another terminal
ros2 run image_tools showimage --ros-args -r /image:=/annotated_image    # view the boxes
```

Example with options: `ros2 run coco_detector coco_detector_node --ros-args -p publish_annotated_image:=False -p device:=cuda -p detection_threshold:=0.7`

Full explanation: [Object detection wiki page](../../docs/wiki/object-detection.md)
