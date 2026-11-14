"""Tests for the YOLO server client (no ROS needed)."""

import base64
from http.server import BaseHTTPRequestHandler, HTTPServer
import threading

import pytest

from object_detection import remote_yolo
from object_detection.remote_yolo import (
    build_request,
    DeadlineCaller,
    Detection,
    parse_detections,
    post_json,
    send_size,
    ServerGate,
)


def chair(box, confidence=0.8, label='chair'):
    return {'detections': [{'label': label, 'confidence': confidence, 'box': box}]}


def test_wide_pictures_are_shrunk_to_640_wide_and_small_ones_sent_as_is():
    assert send_size(1280, 720) == (640, 360)
    assert send_size(1000, 701) == (640, 449)
    assert send_size(640, 480) == (640, 480)
    assert send_size(320, 240) == (320, 240)


def test_request_carries_the_jpeg_threshold_and_classes():
    body = build_request(b'\xff\xd8jpeg', 0.5, ['person', 'chair'])
    assert base64.b64decode(body['image']) == b'\xff\xd8jpeg'
    assert body['conf'] == 0.5
    assert body['classes'] == ['person', 'chair']


def test_boxes_are_scaled_back_to_the_full_picture():
    reply = chair([10, 20, 30, 60])
    assert parse_detections(reply, (640, 360), (1280, 720)) == [
        Detection('chair', 0.8, 20, 40, 60, 120)]


def test_rounding_when_shrinking_does_not_push_boxes_past_the_picture():
    (detection,) = parse_detections(chair([0, 0, 640, 449]), (640, 449), (1000, 701))
    assert (detection.x2, detection.y2) == (1000, 701)


def test_boxes_are_clipped_to_the_picture_and_empty_ones_dropped():
    reply = {'detections': [
        {'label': 'chair', 'confidence': 0.8, 'box': [-5, -5, 1e100, 1e100]},
        {'label': 'cup', 'confidence': 0.8, 'box': [700, 10, 800, 20]},
    ]}
    assert parse_detections(reply, (640, 360), (640, 360)) == [
        Detection('chair', 0.8, 0.0, 0.0, 640, 360)]


def test_no_detections_is_an_empty_list():
    assert parse_detections({'detections': [], 'seconds': 0.01}, (640, 360), (640, 360)) == []


@pytest.mark.parametrize('reply', [
    None,
    {'error': 'GPU out of memory'},
    {'detections': [{'label': 'chair', 'confidence': 0.8}]},
    chair([0, 0, 1, 1], confidence='high'),
    chair([0, 0, 1, 1], confidence=float('nan')),
    chair([0, 0, 1, 1], confidence=1.5),
    chair([0, 0, float('inf'), 1]),
    chair([0, 0, 1, 1], label=''),
    chair([0, 0, 1, 1], label=7),
])
def test_a_reply_the_server_would_not_send_is_rejected(reply):
    with pytest.raises(ValueError):
        parse_detections(reply, (640, 360), (640, 360))


def test_after_a_failure_the_server_is_skipped_for_a_while_then_tried_again():
    gate = ServerGate(retry_sec=30.0)
    assert gate.use_server(0.0)
    assert gate.failed(0.0)            # first failure: worth a warning
    assert not gate.use_server(10.0)
    assert gate.use_server(30.0)
    assert not gate.failed(30.0)       # still down: no second warning
    assert gate.succeeded()            # back: worth a note
    assert not gate.succeeded()
    assert gate.use_server(31.0)


def serve(payload):
    """Start a local server that answers every POST with payload; return its URL."""

    class Handler(BaseHTTPRequestHandler):

        def do_POST(self):
            self.rfile.read(int(self.headers['Content-Length']))
            self.send_response(200)
            self.send_header('Content-Length', str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

        def log_message(self, *args):
            pass

    server = HTTPServer(('127.0.0.1', 0), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server, f'http://127.0.0.1:{server.server_port}/detect'


def test_post_json_returns_the_reply():
    server, url = serve(b'{"detections": []}')
    try:
        assert post_json(url, {'image': ''}, timeout=2.0) == {'detections': []}
    finally:
        server.shutdown()


def test_post_json_refuses_an_oversized_reply(monkeypatch):
    monkeypatch.setattr(remote_yolo, 'MAX_REPLY_BYTES', 10)
    server, url = serve(b'{"detections": [], "padding": "xxxxxxxx"}')
    try:
        with pytest.raises(ValueError):
            post_json(url, {'image': ''}, timeout=2.0)
    finally:
        server.shutdown()


def test_deadline_caller_returns_the_value_or_the_error():
    caller = DeadlineCaller()
    assert caller.call(lambda a, b: a + b, 1, 2, timeout=1.0) == 3
    with pytest.raises(ZeroDivisionError):
        caller.call(lambda: 1 / 0, timeout=1.0)


def test_a_stuck_call_times_out_and_blocks_new_calls_until_it_ends():
    release = threading.Event()
    caller = DeadlineCaller()
    with pytest.raises(TimeoutError):
        caller.call(release.wait, timeout=0.05)
    assert caller.thread.daemon            # never holds up the node's exit
    with pytest.raises(TimeoutError):      # still stuck: fail at once
        caller.call(lambda: 'late', timeout=1.0)
    release.set()
    caller.thread.join(1.0)
    assert caller.call(lambda: 'ok', timeout=1.0) == 'ok'
