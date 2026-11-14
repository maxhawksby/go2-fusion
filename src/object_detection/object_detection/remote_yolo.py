"""
Ask a YOLO server on a GPU machine for boxes, instead of running YOLO here.

The server is host_tools/yolo_server/server.py, reached over an SSH tunnel like
the auto mapper's VLM. Pictures are shrunk before sending (YOLO looks at 640
pixels anyway) and the boxes are scaled back to the full picture.

No ROS here, so it tests on any machine.
"""

import base64
from collections import namedtuple
import json
import math
import threading
import time
import urllib.request

# Width the picture is shrunk to before sending: YOLO's own input size, so
# nothing it would have seen is lost, and each request stays small.
SEND_WIDTH = 640
JPEG_QUALITY = 80
# After the server fails, how long to use the local model before asking it again.
RETRY_SEC = 30.0
# A reply bigger than this is not a list of boxes for one picture.
MAX_REPLY_BYTES = 1024 * 1024
READ_CHUNK_BYTES = 64 * 1024
# Labels longer than this are not class names.
MAX_LABEL_CHARS = 100

Detection = namedtuple('Detection', 'label confidence x1 y1 x2 y2')


def send_size(width, height, send_width=SEND_WIDTH):
    """Return the (width, height) a picture is shrunk to before sending."""
    if width <= send_width:
        return width, height
    return send_width, max(1, round(height * send_width / width))


def build_request(jpeg, min_confidence, classes=()):
    """Build the /detect body from JPEG bytes."""
    return {
        'image': base64.b64encode(jpeg).decode('ascii'),
        'conf': float(min_confidence),
        'classes': list(classes),
    }


def parse_detections(reply, sent_size, full_size):
    """
    Turn the server's reply into Detections in full-picture pixels.

    sent_size and full_size are (width, height) of the picture as sent and as
    taken. Boxes are clipped to the picture; ones left with no area are dropped.
    Raises ValueError on a reply that is not what the server sends.
    """
    if not isinstance(reply, dict) or not isinstance(reply.get('detections'), list):
        raise ValueError(f'unexpected reply: {str(reply)[:200]}')
    (sent_w, sent_h), (full_w, full_h) = sent_size, full_size
    sx, sy = full_w / sent_w, full_h / sent_h
    detections = []
    for entry in reply['detections']:
        try:
            label = entry['label']
            confidence = float(entry['confidence'])
            x1, y1, x2, y2 = (float(v) for v in entry['box'])
        except (KeyError, TypeError, ValueError) as e:
            raise ValueError(f'bad detection {entry!r}: {e}') from e
        if not isinstance(label, str) or not 0 < len(label) <= MAX_LABEL_CHARS:
            raise ValueError(f'bad label in {entry!r}')
        if not all(map(math.isfinite, (x1, y1, x2, y2))) or not 0.0 <= confidence <= 1.0:
            raise ValueError(f'bad detection {entry!r}')
        x1, x2 = (min(max(x * sx, 0.0), full_w) for x in (x1, x2))
        y1, y2 = (min(max(y * sy, 0.0), full_h) for y in (y1, y2))
        if x2 > x1 and y2 > y1:
            detections.append(Detection(label, confidence, x1, y1, x2, y2))
    return detections


def post_json(url, body, timeout):
    """
    POST body as JSON and return the JSON reply.

    The whole exchange is bounded: about timeout seconds plus at most one slow
    socket read, and MAX_REPLY_BYTES of reply.
    """
    deadline = time.monotonic() + timeout
    request = urllib.request.Request(
        url, data=json.dumps(body).encode('utf-8'),
        headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(request, timeout=timeout) as response:
        chunks, size = [], 0
        while True:
            if time.monotonic() > deadline:
                raise TimeoutError(f'no full reply within {timeout} s')
            chunk = response.read(READ_CHUNK_BYTES)
            if not chunk:
                break
            size += len(chunk)
            if size > MAX_REPLY_BYTES:
                raise ValueError(f'reply over {MAX_REPLY_BYTES} bytes')
            chunks.append(chunk)
    return json.loads(b''.join(chunks).decode('utf-8'))


class DeadlineCaller:
    """
    Run one call at a time on a daemon thread and wait for it at most timeout.

    A call that overruns keeps running in the background (Python can't stop a
    thread), but as a daemon it never holds up the node's exit, and while it
    runs every new call fails at once instead of piling up more threads.
    """

    def __init__(self):
        self.thread = None

    def call(self, fn, *args, timeout):
        if self.thread is not None and self.thread.is_alive():
            raise TimeoutError('an earlier request is still waiting for the server')
        outcome = {}

        def run():
            try:
                outcome['value'] = fn(*args)
            except BaseException as e:   # handed to the caller below
                outcome['error'] = e

        self.thread = threading.Thread(target=run, daemon=True)
        self.thread.start()
        self.thread.join(timeout)
        if self.thread.is_alive():
            raise TimeoutError(f'no reply within {timeout} s')
        if 'error' in outcome:
            raise outcome['error']
        return outcome['value']


class ServerGate:
    """Whether to ask the server now: not for RETRY_SEC after it failed."""

    def __init__(self, retry_sec=RETRY_SEC):
        self.retry_sec = retry_sec
        self.retry_after = None

    def use_server(self, now):
        return self.retry_after is None or now >= self.retry_after

    def failed(self, now):
        """Note a failure; True if the server had been working until now."""
        was_working = self.retry_after is None
        self.retry_after = now + self.retry_sec
        return was_working

    def succeeded(self):
        """Note a success; True if the server had been failing until now."""
        was_failing = self.retry_after is not None
        self.retry_after = None
        return was_failing
