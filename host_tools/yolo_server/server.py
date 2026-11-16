"""HTTP server around YOLO on a GPU, so the robot laptop needs no GPU.

POST /detect  {"image": base64 JPEG, "conf": minimum confidence 0 to 1 (default 0.25),
               "classes": [names] (YOLOE weights only; [] keeps the model's own)}
           -> {"detections": [{"label", "confidence", "box": [x1, y1, x2, y2] pixels}],
               "seconds": s}
GET  /health  -> {"model": weights file, "device": "cuda:N", "gpu_gb": memory in use}

The object_detection node (inference_url, or YOLO_URL) sends one picture per
request and does everything else (positions, memory, people) itself. It has no
password: keep it on localhost and reach it over an SSH tunnel.
"""

import base64
import io
import json
import math
import os
import socket
import sys
import threading
import time
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import torch
from PIL import Image
from ultralytics import YOLO

MODEL_PATH = os.environ.get('YOLO_MODEL', 'yolov8n.pt')
PORT = int(os.environ.get('YOLO_PORT', '8200'))
DEVICE = os.environ.get('YOLO_DEVICE', 'cuda:0' if torch.cuda.is_available() else 'cpu')
# Limits, so one bad or hostile request can't use up memory or threads. A
# 640-wide JPEG is about 50 KB; the robot camera is 1280 x 720.
MAX_BODY_BYTES = 8 * 1024 * 1024
MAX_PIXELS = 4096 * 2160
MAX_CLASSES = 50
MAX_CLASS_CHARS = 64
# Requests decoded or waiting for the GPU at once; more are refused (503).
MAX_IN_FLIGHT = 4
# Seconds a client may wait between bytes, and to send its whole body.
SOCKET_TIMEOUT_SEC = 10.0
# Open connections at once; more are closed before a thread is started.
MAX_CONNECTIONS = 8
# Every connection is closed this long after it was accepted, finished or not.
CONNECTION_DEADLINE_SEC = 15.0


class BoundedServer(ThreadingHTTPServer):
    """A threaded server that refuses connections beyond MAX_CONNECTIONS."""

    daemon_threads = True

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.connections = threading.BoundedSemaphore(MAX_CONNECTIONS)

    def process_request(self, request, client_address):
        if not self.connections.acquire(blocking=False):
            self.shutdown_request(request)
            return
        try:
            super().process_request(request, client_address)
        except Exception:
            self.connections.release()
            raise

    def handle_error(self, request, client_address):
        # Usually the watchdog below closing a slow client: one line, not a traceback.
        print(f'Dropped the connection from {client_address[0]}: {sys.exc_info()[1]!r}',
              flush=True)

    def process_request_thread(self, request, client_address):
        # Whatever the client does (trickling headers or body, never reading
        # the reply), its connection is closed CONNECTION_DEADLINE_SEC after
        # it was accepted, so no client can hold a slot for long.
        watchdog = threading.Timer(CONNECTION_DEADLINE_SEC, close_socket, (request,))
        watchdog.daemon = True
        watchdog.start()
        try:
            super().process_request_thread(request, client_address)
        finally:
            watchdog.cancel()
            self.connections.release()


def close_socket(sock):
    try:
        sock.shutdown(socket.SHUT_RDWR)
    except OSError:
        pass   # already closed


def read_body(stream, sock, length, timeout):
    """Read exactly length bytes from stream, taking at most timeout seconds in all."""
    deadline = time.monotonic() + timeout
    chunks, left = [], length
    while left > 0:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise ValueError(f'body not received within {timeout} s')
        sock.settimeout(remaining)
        try:
            chunk = stream.read1(min(left, 65536))
        except socket.timeout:
            raise ValueError(f'body not received within {timeout} s') from None
        if not chunk:
            raise ValueError('body ended early')
        chunks.append(chunk)
        left -= len(chunk)
    if time.monotonic() > deadline:
        raise ValueError(f'body not received within {timeout} s')
    return b''.join(chunks)


def decode_image(text):
    """Decode a base64 JPEG or PNG, checking its size before decompressing it."""
    if not isinstance(text, str):
        raise ValueError('image must be a base64 string')
    data = base64.b64decode(text, validate=True)
    if not data:
        raise ValueError('image is empty')
    try:
        width, height = Image.open(io.BytesIO(data)).size   # reads the header only
    except Exception as e:
        raise ValueError(f'image is not a JPEG or PNG: {e}') from e
    if width * height > MAX_PIXELS:
        raise ValueError(f'image is {width} x {height}; at most {MAX_PIXELS} pixels')
    frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        raise ValueError('image could not be decoded')
    return frame


def parse_request(request):
    """Check a /detect body; return (frame, conf, classes) or raise ValueError."""
    if not isinstance(request, dict):
        raise ValueError('body must be a JSON object')
    conf = float(request.get('conf', 0.25))
    if not (math.isfinite(conf) and 0.0 <= conf <= 1.0):
        raise ValueError('conf must be between 0 and 1')
    classes = request.get('classes', [])
    if not isinstance(classes, list) or len(classes) > MAX_CLASSES:
        raise ValueError(f'classes must be a list of at most {MAX_CLASSES} names')
    classes = [c.strip() if isinstance(c, str) else '' for c in classes]
    if not all(0 < len(c) <= MAX_CLASS_CHARS for c in classes):
        raise ValueError(f'each class must be a name of 1 to {MAX_CLASS_CHARS} characters')
    return decode_image(request.get('image')), conf, classes


class Service:

    def __init__(self):
        self.model = YOLO(MODEL_PATH)
        self.classes = []
        # One picture at a time on the GPU; set_classes changes the model.
        self.lock = threading.Lock()
        # Warm up, so the first real request is not slow.
        self.model(np.zeros((360, 640, 3), np.uint8), device=DEVICE, verbose=False)

    def use_classes(self, classes):
        """Switch the vocabulary; call with the lock held."""
        if classes == self.classes:
            return
        if classes and not hasattr(self.model, 'set_classes'):
            raise ValueError(f'{MODEL_PATH} has fixed classes; classes needs YOLOE weights')
        if classes:
            self.model.set_classes(classes)
        else:
            # set_classes replaced the model's own names; only a reload brings them back.
            self.model = YOLO(MODEL_PATH)
        self.classes = classes

    def detect(self, request):
        frame, conf, classes = parse_request(request)
        with self.lock:
            self.use_classes(classes)
            result = self.model(frame, conf=conf, device=DEVICE, verbose=False)[0]
        names = result.names
        return {'detections': [
            {'label': names[int(c)], 'confidence': round(float(p), 3),
             'box': [round(float(v), 1) for v in xyxy]}
            for c, p, xyxy in zip(result.boxes.cls, result.boxes.conf, result.boxes.xyxy)
        ]}


def make_handler(service):
    in_flight = threading.BoundedSemaphore(MAX_IN_FLIGHT)

    class Handler(BaseHTTPRequestHandler):

        timeout = SOCKET_TIMEOUT_SEC

        def reply(self, code, data):
            payload = json.dumps(data).encode()
            self.send_response(code)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

        def do_GET(self):
            if self.path != '/health':
                self.reply(404, {'error': 'unknown path'})
                return
            gpu_gb = torch.cuda.memory_allocated() / 1e9 if DEVICE.startswith('cuda') else 0.0
            self.reply(200, {'model': MODEL_PATH, 'device': DEVICE, 'gpu_gb': round(gpu_gb, 2)})

        def do_POST(self):
            if self.path != '/detect':
                self.reply(404, {'error': 'unknown path'})
                return
            if not in_flight.acquire(blocking=False):
                self.reply(503, {'error': f'busy: {MAX_IN_FLIGHT} requests already running'})
                return
            started = time.monotonic()
            try:
                length = int(self.headers.get('Content-Length', 0))
                if not 0 < length <= MAX_BODY_BYTES:
                    raise ValueError(f'body must be 1 to {MAX_BODY_BYTES} bytes')
                try:
                    body = read_body(self.rfile, self.connection, length, SOCKET_TIMEOUT_SEC)
                finally:
                    # read_body shortens the timeout; the reply gets the usual one.
                    self.connection.settimeout(SOCKET_TIMEOUT_SEC)
                request = json.loads(body)
                self.reply(200, {**service.detect(request),
                                 'seconds': round(time.monotonic() - started, 4)})
            except (KeyError, TypeError, ValueError) as e:
                self.reply(400, {'error': str(e)})
            except OSError:
                return   # the client left, or the watchdog closed it: nobody to answer
            except torch.cuda.OutOfMemoryError:
                torch.cuda.empty_cache()
                self.reply(503, {'error': 'GPU out of memory'})
            except Exception as e:   # one bad request must not take the server down
                traceback.print_exc()
                self.reply(500, {'error': f'{type(e).__name__}: {e}'})
            finally:
                in_flight.release()

        def log_message(self, *args):
            pass

    return Handler


if __name__ == '__main__':
    service = Service()
    print(f'YOLO {MODEL_PATH} ready on {DEVICE}, port {PORT}', flush=True)
    # Localhost by default: this has no password, so reach it over an SSH tunnel.
    host = os.environ.get('YOLO_HOST', '127.0.0.1')
    BoundedServer((host, PORT), make_handler(service)).serve_forever()
