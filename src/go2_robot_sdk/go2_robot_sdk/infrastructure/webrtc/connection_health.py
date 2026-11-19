"""Detects a WebRTC data channel that went quiet without closing.

Seen in the lab: the camera video (its own media track) kept arriving while
odometry, IMU, joint states and the lidar cloud (all on the data channel)
stopped. Nothing closed and no connection state changed, so the driver kept
running as if connected and received nothing: no cloud, no /scan, no map.
"""

# The robot sends odometry at about 15 Hz and state topics continuously, so
# this long without a single data-channel message means the channel stalled.
DATA_STALL_TIMEOUT_SEC = 5.0


def data_stalled(last_message_mono, now_mono, timeout=DATA_STALL_TIMEOUT_SEC):
    """True if the data channel has been silent for longer than `timeout`.

    `last_message_mono` is when the last message arrived (monotonic
    seconds), or None before the first one: a connection still being set up
    hasn't stalled, it just hasn't started.
    """

    if last_message_mono is None:
        return False
    return now_mono - last_message_mono > timeout


# One stream (the lidar cloud, the camera) can go quiet while the rest of
# the data channel keeps flowing, e.g. after a reconnect: odometry came back,
# the cloud and the camera didn't. The data-channel check above can't see
# that, since messages still arrive.
STREAM_STALL_SEC = 10.0

OK, NUDGE, GIVE_UP = 'ok', 'nudge', 'give_up'


class StreamWatch:
    """Liveness of one stream that should be flowing.

    After `stall_sec` of silence (counting from start() when nothing has
    arrived yet), check() returns NUDGE once: re-send whatever request makes
    the robot send this stream. If it is still silent `stall_sec` after that,
    check() returns GIVE_UP; the caller decides what that means (reconnect,
    or nudge again). Any arrival resets it.
    """

    def __init__(self, stall_sec=STREAM_STALL_SEC):
        self.stall_sec = stall_sec
        self.started = None
        self.last_seen = None
        self.nudged_at = None
        self.count = 0

    def start(self, now):
        """The connection is up and this stream should begin."""
        self.started = now
        self.last_seen = None
        self.nudged_at = None

    def seen(self, now):
        self.last_seen = now
        self.nudged_at = None
        self.count += 1

    def check(self, now):
        if self.started is None:
            return OK
        reference = self.last_seen if self.last_seen is not None else self.started
        if now - reference <= self.stall_sec:
            return OK
        if self.nudged_at is None:
            self.nudged_at = now
            return NUDGE
        if now - self.nudged_at > self.stall_sec:
            self.nudged_at = now
            return GIVE_UP
        return OK

    def take_count(self):
        """Messages since the last call, for rate logging."""
        count, self.count = self.count, 0
        return count
