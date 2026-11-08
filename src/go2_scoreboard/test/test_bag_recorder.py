from go2_scoreboard.bag_recorder import BagRecorder


class FinishedProcess:
    pid = 123
    returncode = 0

    def poll(self):
        return self.returncode


class RunningProcess(FinishedProcess):
    returncode = None


def test_bag_command_contains_required_topics_and_cleans_up(tmp_path):
    calls = []

    def popen(command, **kwargs):
        calls.append((command, kwargs))
        return FinishedProcess()

    recorder = BagRecorder(str(tmp_path / 'bag'), popen=popen)
    recorder.start()
    assert calls[0][0][:5] == ['ros2', 'bag', 'record', '-o', str(tmp_path / 'bag')]
    assert '/map' in calls[0][0]
    assert '/local_costmap/costmap' in calls[0][0]
    assert '/auto_mapper/events' in calls[0][0]
    assert '/scoreboard/status' in calls[0][0]
    assert '--qos-profile-overrides-path' in calls[0][0]
    assert not recorder.healthy()
    recorder.stop()
    assert recorder.process is None


def test_bag_ready_requires_recording_and_odom_subscription(tmp_path):
    recorder = BagRecorder(
        str(tmp_path / 'bag'), popen=lambda *_args, **_kwargs: RunningProcess())
    recorder.start()
    assert not recorder.ready()
    with open(recorder.log_path, 'a', encoding='utf-8') as stream:
        stream.write("Recording...\nSubscribed to topic '/odom'\n")
    assert recorder.ready()
    recorder.process.returncode = 0
    recorder.stop()
