# lidar_processor_cpp

C++ (PCL) drop-in versions of `lidar_processor`'s two nodes, with the same topics and parameters: `lidar_to_pointcloud_node` builds up `/pointcloud/aggregated`, `pointcloud_aggregator_node` publishes `/pointcloud/filtered` and `/pointcloud/downsampled`.

## Build and run

Inside the container:

```bash
cbs lidar_processor_cpp      # colcon build --packages-select lidar_processor_cpp
ros2 launch lidar_processor_cpp lidar_to_pointcloud.launch.py
ros2 launch lidar_processor_cpp pointcloud_aggregator.launch.py
ros2 launch lidar_processor_cpp lidar_processing_pipeline.launch.py    # both together
ros2 launch lidar_processor_cpp lidar_processing_pipeline.launch.py \
    map_name:="my_custom_map" max_range:=15.0 publish_rate:=10.0
```

License: BSD-3-Clause.

Full explanation, including every parameter: [Lidar wiki page](../../docs/wiki/lidar.md)
