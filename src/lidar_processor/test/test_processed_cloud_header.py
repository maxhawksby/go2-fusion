"""Exercise the real cloud-construction path without starting a ROS node."""

from types import SimpleNamespace as NS

from builtin_interfaces.msg import Time
import numpy as np
import pytest
from std_msgs.msg import Header

from lidar_processor.pointcloud_aggregator_node import PointCloudAggregatorNode


@pytest.mark.parametrize('stride', [1, 2])
def test_processed_clouds_have_real_headers_and_expected_points(stride):
    filtered, downsampled, errors = [], [], []
    points = np.array([[1., 2., 3.], [4., 5., 6.], [7., 8., 9.]], dtype=np.float32)
    node = NS(latest_points=points, config=NS(downsample_rate=stride),
              get_clock=lambda: NS(now=lambda: NS(to_msg=lambda: Time(sec=123, nanosec=456))),
              filtered_pub=NS(publish=filtered.append), downsampled_pub=NS(publish=downsampled.append),
              last_publish_time=0., get_logger=lambda: NS(error=errors.append, info=lambda _text: None))
    PointCloudAggregatorNode._publish_callback(node)
    assert not errors and len(filtered) == 1
    message = filtered[0]
    assert isinstance(message.header, Header)
    assert message.header.frame_id == 'base_link'
    assert message.header.stamp.sec == 123 and message.header.stamp.nanosec == 456
    assert message.width == 3
    if stride > 1:
        assert len(downsampled) == 1 and downsampled[0].width == 2
        assert downsampled[0].header == message.header
    else:
        assert not downsampled
