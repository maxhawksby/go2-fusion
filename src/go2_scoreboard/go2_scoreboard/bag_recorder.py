"""Managed ros2 bag recording for live scoreboard runs."""

import os
import signal
import subprocess
import time


DEFAULT_TOPICS = (
    '/map', '/odom', '/scan', '/local_costmap/costmap', '/cmd_vel_out',
    '/auto_mapper/events',
    '/scoreboard/status', '/navigate_to_pose/_action/status', '/tf', '/tf_static',
    '/rosout', '/clock', '/sim/bumper',
)


class BagRecorder:
    def __init__(self, output_dir, topics=DEFAULT_TOPICS, popen=subprocess.Popen):
        self.output_dir = output_dir
        self.topics = tuple(topics)
        self._popen = popen
        self.process = None
        self.log_stream = None
        parent = os.path.dirname(self.output_dir)
        self.log_path = os.path.join(parent, 'bag.log')
        self.qos_path = os.path.join(parent, 'bag_qos_overrides.yaml')

    def start(self):
        if self.process is not None:
            return
        os.makedirs(os.path.dirname(self.output_dir), exist_ok=True)
        with open(self.qos_path, 'w', encoding='utf-8') as stream:
            stream.write(
                '/auto_mapper/events:\n'
                '  history: keep_last\n'
                '  depth: 100\n'
                '  reliability: reliable\n'
                '  durability: transient_local\n')
        self.log_stream = open(self.log_path, 'w', encoding='utf-8')
        command = [
            'ros2', 'bag', 'record', '-o', self.output_dir,
            '--qos-profile-overrides-path', self.qos_path, *self.topics,
        ]
        self.process = self._popen(command, stdout=self.log_stream,
                                   stderr=subprocess.STDOUT, start_new_session=True)

    def healthy(self):
        return self.process is not None and self.process.poll() is None

    def ready(self):
        if not self.healthy():
            return False
        try:
            with open(self.log_path, encoding='utf-8') as stream:
                log = stream.read()
        except OSError:
            return False
        return 'Recording...' in log and "Subscribed to topic '/odom'" in log

    def wait_ready(self, timeout=15.0, poll_interval=0.05):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.ready():
                return True
            if not self.healthy():
                return False
            time.sleep(poll_interval)
        return self.ready()

    def stop(self, timeout=15.0):
        process = self.process
        if process is None:
            return
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGINT)
                process.wait(timeout=timeout)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                if process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=5.0)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=5.0)
        if self.log_stream is not None:
            self.log_stream.close()
        self.process = None
        self.log_stream = None
