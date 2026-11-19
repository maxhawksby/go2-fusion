"""Exercise live callbacks without creating nodes or publishing messages."""

from types import SimpleNamespace

import numpy as np
from nav_msgs.msg import OccupancyGrid, Odometry

from go2_scoreboard import map_metrics
from go2_scoreboard.accumulator import RunAccumulator
from go2_scoreboard.recorder import ScoreboardRecorder


def test_map_callbacks_keep_prepared_geometry_between_odometry_updates(monkeypatch):
    truth = map_metrics.Grid(np.zeros((4, 4), dtype=np.int16), 0.5, 0.0, 0.0)
    run = RunAccumulator('callback-test')
    run.start(1.0)
    recorder = SimpleNamespace(
        accumulator=run, ground_truth=truth, start_pose=(0.0, 0.0, 0.0),
        completeness_scorer=map_metrics.CompletenessScorer(truth, (0.0, 0.0, 0.0)),
        _now_seconds=lambda: 1.0,
    )

    def unexpected_rebuild(_grid):
        raise AssertionError('a live callback rebuilt the answer-key floor mask')

    monkeypatch.setattr(map_metrics, '_interior_mask', unexpected_rebuild)
    grid = OccupancyGrid()
    grid.info.width = grid.info.height = 4
    grid.info.resolution = 0.5
    grid.data = [0] * 16
    odom = Odometry()
    for index in range(201):
        odom.pose.pose.position.x = index * 0.1
        ScoreboardRecorder._on_odom(recorder, odom)
        if index % 10 == 0:
            ScoreboardRecorder._on_map(recorder, grid)
    assert abs(run.distance_m - 20.0) < 1e-9
    assert run.completeness_pct == 100.0
    assert run.warnings == []
