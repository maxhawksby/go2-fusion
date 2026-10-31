import asyncio
import time
from types import SimpleNamespace

from go2_robot_sdk.infrastructure.webrtc.connection_health import (
    DATA_STALL_TIMEOUT_SEC,
    data_stalled,
)


def test_no_message_yet_is_not_a_stall():
    # Still connecting: nothing has arrived, but nothing has stopped either.
    assert not data_stalled(None, 1000.0)


def test_recent_message_is_not_a_stall():
    assert not data_stalled(100.0, 100.0 + DATA_STALL_TIMEOUT_SEC - 0.1)


def test_silence_past_the_timeout_is_a_stall():
    assert data_stalled(100.0, 100.0 + DATA_STALL_TIMEOUT_SEC + 0.1)


def _adapter_with(last_message_mono):
    """A WebRTCAdapter holding one fake connection for robot '0'. Needs the
    driver's dependencies (aiortc), so run inside the Docker container."""
    from go2_robot_sdk.infrastructure.webrtc.webrtc_adapter import WebRTCAdapter

    adapter = WebRTCAdapter.__new__(WebRTCAdapter)
    adapter.connections = {'0': SimpleNamespace(last_message_mono=last_message_mono)}
    adapter.disconnected_events = {'0': asyncio.Event()}
    adapter.lidar_watch, adapter.video_watch, adapter.stream_log_at = {}, {}, {}
    return adapter


def test_adapter_reports_a_stalled_channel_as_lost():
    adapter = _adapter_with(time.monotonic() - DATA_STALL_TIMEOUT_SEC - 1.0)
    assert not adapter.is_connected('0')
    # Latched, so the retry logic sees it even if a message arrives late.
    adapter.connections['0'].last_message_mono = time.monotonic()
    assert not adapter.is_connected('0')


def test_adapter_keeps_a_live_or_starting_channel():
    assert _adapter_with(time.monotonic()).is_connected('0')
    assert _adapter_with(None).is_connected('0')


def test_stream_watch_nudges_then_gives_up():
    from go2_robot_sdk.infrastructure.webrtc.connection_health import (
        GIVE_UP, NUDGE, OK, StreamWatch)
    w = StreamWatch(stall_sec=10.0)
    assert w.check(0.0) == OK                  # not started: nothing expected
    w.start(100.0)
    assert w.check(105.0) == OK                # still within the grace period
    assert w.check(111.0) == NUDGE             # never arrived: nudge once
    assert w.check(115.0) == OK                # waiting for the nudge to work
    assert w.check(122.0) == GIVE_UP           # still nothing: give up
    assert w.check(125.0) == OK                # ...and wait again before repeating
    w.seen(126.0)                              # it came back
    assert w.check(130.0) == OK
    assert w.check(137.0) == NUDGE             # silent again 11 s later


def test_stream_watch_counts_for_rate_logging():
    from go2_robot_sdk.infrastructure.webrtc.connection_health import StreamWatch
    w = StreamWatch()
    w.start(0.0)
    for t in range(5):
        w.seen(float(t))
    assert w.take_count() == 5
    assert w.take_count() == 0


class _FakeChannel:
    def __init__(self):
        self.sent = []

    def send(self, text):
        self.sent.append(text)


class _FakeConn:
    def __init__(self):
        self.data_channel = _FakeChannel()
        self.published = []
        self.last_message_mono = time.monotonic()

    def publish(self, topic, data, msg_type='msg'):
        self.published.append((topic, data, msg_type))


def _validated_adapter(enable_video=True):
    from go2_robot_sdk.infrastructure.webrtc.webrtc_adapter import WebRTCAdapter

    adapter = WebRTCAdapter.__new__(WebRTCAdapter)
    adapter.config = SimpleNamespace(enable_video=enable_video)
    adapter.connections = {'0': _FakeConn()}
    adapter.disconnected_events = {'0': asyncio.Event()}
    adapter.on_validated_callback = None
    adapter.lidar_watch, adapter.video_watch, adapter.stream_log_at = {}, {}, {}
    adapter._on_validated('0')
    return adapter, adapter.connections['0']


def test_validation_asks_for_lidar_once_the_channel_is_open():
    _, conn = _validated_adapter()
    assert ('', {'req_type': 'disable_traffic_saving', 'instruction': 'on'},
            'rtc_inner_req') in conn.published
    assert ('rt/utlidar/switch', 'on', 'msg') in conn.published


def test_quiet_lidar_is_nudged_then_reconnects():
    adapter, conn = _validated_adapter(enable_video=False)
    watch = adapter.lidar_watch['0']
    conn.published.clear()
    watch.started -= 11.0                                  # 11 s, no lidar
    assert adapter.is_connected('0')                       # nudge, keep going
    assert ('rt/utlidar/switch', 'on', 'msg') in conn.published
    watch.nudged_at -= 11.0                                # still nothing
    conn.last_message_mono = time.monotonic()              # other data flows
    assert not adapter.is_connected('0')                   # reconnect


def test_arriving_lidar_keeps_the_connection():
    adapter, conn = _validated_adapter(enable_video=False)
    adapter.lidar_watch['0'].started -= 11.0
    adapter._on_data_channel_message(None, {'topic': 'rt/utlidar/voxel_map_compressed'}, '0')
    conn.published.clear()
    assert adapter.is_connected('0') and not conn.published


def test_quiet_camera_is_nudged_but_never_reconnects():
    adapter, conn = _validated_adapter()
    adapter._on_data_channel_message(None, {'topic': 'rt/utlidar/voxel_map_compressed'}, '0')
    video = adapter.video_watch['0']
    conn.published.clear()
    video.started -= 11.0
    assert adapter.is_connected('0')
    assert ('', 'on', 'vid') in conn.published
    video.nudged_at -= 11.0
    adapter._on_data_channel_message(None, {'topic': 'rt/utlidar/voxel_map_compressed'}, '0')
    assert adapter.is_connected('0')                       # still connected
    adapter.note_video_frame('0')                          # frames are back
    assert video.check(time.monotonic()) == 'ok'
