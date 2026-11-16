import csv
import json
import os

import numpy as np

from go2_scoreboard.accumulator import RunAccumulator
from go2_scoreboard.artifacts import write_artifacts
from go2_scoreboard.map_metrics import Grid


def test_all_required_artifacts_and_index_are_written(tmp_path):
    run = RunAccumulator('fixed-run', source='sim', run_label='frontier')
    run.handle_event('exploration_started', 0, 1.0)
    run.update_map(Grid(np.zeros((2, 3), dtype=np.int16), 0.5, 0.0, 0.0), 2.0)
    run.sample_timeline()
    run.begin_finalizing()
    run.finalize('complete')
    run_dir = tmp_path / run.run_id
    write_artifacts(str(run_dir), run)

    expected = {
        'run.json', 'scorecard.csv', 'report.md', 'timeline.csv',
        'resources.csv', 'final_map.yaml', 'final_map.pgm',
    }
    assert expected <= set(os.listdir(run_dir))
    with open(run_dir / 'run.json', encoding='utf-8') as stream:
        assert json.load(stream)['known_area_m2'] == 1.5
    with open(tmp_path / 'index.csv', newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    assert [row['run_id'] for row in rows] == ['fixed-run']

    # A repeated final write replaces the row rather than duplicating it.
    write_artifacts(str(run_dir), run)
    with open(tmp_path / 'index.csv', newline='', encoding='utf-8') as stream:
        assert len(list(csv.DictReader(stream))) == 1
