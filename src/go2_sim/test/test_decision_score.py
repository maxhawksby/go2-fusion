"""Tests for scoring VLM decisions against the true map (no ROS needed)."""

import json
import math
import os

import numpy as np
import pytest

from go2_sim import world_tools
from go2_sim.decision_score import (
    map_to_world,
    score_decision,
    score_run,
    summarize,
    true_reveal,
    world_to_map,
)

RES = 0.05


def building():
    """A 10 x 10 m hall with a closed 2 x 2 m box at (6..8, 6..8)."""
    cells = int(10 / RES)
    data = np.full((cells, cells), world_tools.FREE, dtype=np.uint8)
    data[0, :] = data[-1, :] = data[:, 0] = data[:, -1] = world_tools.OCCUPIED
    lo, hi = int(6 / RES), int(8 / RES)
    data[lo, lo:hi + 1] = data[hi, lo:hi + 1] = world_tools.OCCUPIED
    data[lo:hi + 1, lo] = data[lo:hi + 1, hi] = world_tools.OCCUPIED
    return world_tools.Grid(data, RES, 0.0, 0.0)


def unknown_snapshot():
    """A map-frame snapshot where nothing is known yet."""
    return np.full((10, 10), -1, dtype=np.int8), RES, (-100.0, -100.0)


def test_map_and_world_frames_round_trip_with_a_turned_start():
    start = (2.0, 3.0, math.pi / 2)
    assert map_to_world(1.0, 0.0, start) == pytest.approx((2.0, 4.0))
    assert world_to_map(*map_to_world(0.7, -1.2, start), start) == pytest.approx((0.7, -1.2))


def test_walls_limit_what_a_sweep_reveals():
    gt, start = building(), (0.0, 0.0, 0.0)
    in_box = true_reveal(gt, unknown_snapshot(), start, 7.0, 7.0)
    in_hall = true_reveal(gt, unknown_snapshot(), start, 3.0, 3.0)
    box_cells = (int(2 / RES) + 1) ** 2
    assert 0 < in_box <= box_cells
    assert in_hall > 3 * in_box


def test_cells_already_known_are_not_counted():
    gt, start = building(), (0.0, 0.0, 0.0)
    known = (np.zeros((int(10 / RES), int(10 / RES)), dtype=np.int8), RES, (0.0, 0.0))
    assert true_reveal(gt, known, start, 3.0, 3.0) == 0


def test_a_decision_gets_true_values_for_both_picks():
    record = {
        'candidates': [{'x': 7.0, 'y': 7.0, 'distance_m': 1.0},
                       {'x': 3.0, 'y': 3.0, 'distance_m': 1.0}],
        'map_pick': 0, 'vlm_pick': 1,
    }
    score = score_decision(record, unknown_snapshot(), building(), (0.0, 0.0, 0.0))
    assert score['vlm'] > score['map']
    assert score['best'] == score['vlm']
    assert not score['agree']


def test_a_run_folder_is_scored_end_to_end(tmp_path):
    run = tmp_path / 'seed_28000'
    trial = run / '20261109-150000'
    nav = run / 'nav'
    trial.mkdir(parents=True)
    nav.mkdir()
    world_tools.write_map(building(), str(trial / 'placeholder_rooms.gt.yaml'))
    with open(trial / 'results.csv', 'w') as f:
        f.write('run,seed,start_x,start_y,start_yaw\n1,28000,0.0,0.0,0.0\n')
    data, res, origin = unknown_snapshot()
    np.savez_compressed(nav / 'map_0001.npz', data=data, resolution=res, origin=origin)
    record = {
        'decision': 1, 'map': 'map_0001.npz', 'map_pick': 0, 'vlm_pick': 1,
        'candidates': [{'x': 7.0, 'y': 7.0, 'distance_m': 1.0},
                       {'x': 3.0, 'y': 3.0, 'distance_m': 1.5}],
    }
    with open(nav / 'decisions.jsonl', 'w') as f:
        f.write(json.dumps(record) + '\n')
    scores = score_run(str(run))
    assert len(scores) == 1 and scores[0]['vlm'] > scores[0]['map']
    text = summarize(scores)
    assert 'VLM picked differently in 1' in text
    assert 'VLM better 1, worse 0' in text


def test_a_run_without_decisions_scores_nothing(tmp_path):
    assert score_run(str(tmp_path)) == []
    assert summarize([]) == 'No scored decisions.'
    assert os.path.isdir(tmp_path)
