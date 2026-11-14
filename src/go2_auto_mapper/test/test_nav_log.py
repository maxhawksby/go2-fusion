import json
import os

from go2_auto_mapper.nav_log import NavLog, known_cell_count


def read_goals(folder):
    with open(os.path.join(folder, 'goals.jsonl')) as f:
        return [json.loads(line) for line in f]


def test_a_goal_is_labeled_with_what_it_revealed(tmp_path):
    log = NavLog(str(tmp_path))
    log.goal_sent((0.0, 0.0, 0.0), (3.0, 1.0), 1000, 12, -500.0)
    log.goal_ended('reached')
    log.finish(1450)
    goal = read_goals(tmp_path)[0]
    assert goal['target'] == [3.0, 1.0]
    assert goal['result'] == 'reached'
    assert goal['revealed_cells'] == 450


def test_the_next_goal_finishes_the_one_before(tmp_path):
    log = NavLog(str(tmp_path))
    log.goal_sent((0.0, 0.0, 0.0), (1.0, 0.0), 100, 5, None)
    log.goal_ended('failed')
    log.goal_sent((0.0, 0.0, 0.0), (0.0, 1.0), 100, 5, None)
    goals = read_goals(tmp_path)
    assert len(goals) == 1
    assert goals[0]['result'] == 'failed'
    assert goals[0]['revealed_cells'] == 0


def test_getting_stuck_is_recorded_on_the_goal(tmp_path):
    log = NavLog(str(tmp_path))
    log.goal_sent((0.0, 0.0, 0.0), (1.0, 0.0), 100, 5, None)
    log.event('backed up')
    log.event('stuck: 3 goals failed in a row')
    log.finish(100)
    assert read_goals(tmp_path)[0]['events'] == ['backed up', 'stuck: 3 goals failed in a row']


def test_the_picture_is_saved_and_named_on_the_goal(tmp_path):
    log = NavLog(str(tmp_path))
    log.goal_sent((0.0, 0.0, 0.0), (1.0, 0.0), 100, 5, None, jpeg=b'picture')
    log.finish(100)
    name = read_goals(tmp_path)[0]['image']
    assert (tmp_path / name).read_bytes() == b'picture'


def test_the_first_result_of_a_goal_is_kept(tmp_path):
    log = NavLog(str(tmp_path))
    log.goal_sent((0.0, 0.0, 0.0), (1.0, 0.0), 100, 5, None)
    log.goal_ended('reached')
    log.goal_ended('failed')
    log.finish(100)
    assert read_goals(tmp_path)[0]['result'] == 'reached'


def test_events_with_no_goal_are_ignored(tmp_path):
    log = NavLog(str(tmp_path))
    log.event('backed up')
    log.goal_ended('reached')
    log.finish(100)
    assert not (tmp_path / 'goals.jsonl').exists()


def test_a_folder_it_cannot_make_only_warns(tmp_path):
    blocker = tmp_path / 'file'
    blocker.write_text('x')
    warnings = []
    log = NavLog(str(blocker / 'sub'), warn=warnings.append)
    log.goal_sent((0.0, 0.0, 0.0), (1.0, 0.0), 100, 5, None, jpeg=b'picture')
    log.finish(100)
    assert warnings


def test_known_cells_skip_unknown():
    assert known_cell_count([-1, 0, 100, -1, 50]) == 3


def test_a_vlm_ranking_is_logged_with_both_picks_and_the_map(tmp_path):
    import numpy as np
    log = NavLog(str(tmp_path))
    candidates = [
        {'x': 1.0, 'y': 0.0, 'map_score': -300.0, 'vlm_score': 0.2, 'distance_m': 1.0},
        {'x': 0.0, 'y': 2.0, 'map_score': -250.0, 'vlm_score': 0.9, 'distance_m': 2.0},
    ]
    grid = (0.05, -1.0, -2.0, 3, 2, [-1, 0, 100, 0, -1, 0])
    log.decision((0.0, 0.0), candidates, 0, 1, 'shadow', grid)
    with open(os.path.join(tmp_path, 'decisions.jsonl')) as f:
        (record,) = [json.loads(line) for line in f]
    assert (record['map_pick'], record['vlm_pick'], record['mode']) == (0, 1, 'shadow')
    assert record['candidates'][1]['vlm_score'] == 0.9
    saved = np.load(os.path.join(tmp_path, record['map']))
    assert saved['data'].shape == (2, 3)
    assert saved['data'][0, 2] == 100
    assert float(saved['resolution']) == 0.05


def test_a_restart_into_the_same_folder_keeps_the_earlier_maps(tmp_path):
    import numpy as np
    candidates = [{'x': 1.0, 'y': 0.0, 'map_score': 0.0, 'vlm_score': 0.5, 'distance_m': 1.0}]
    first, second = NavLog(str(tmp_path)), NavLog(str(tmp_path))   # explorer started twice
    first.decision((0.0, 0.0), candidates, 0, 0, 'shadow', (0.05, 0.0, 0.0, 1, 1, [0]))
    second.decision((0.0, 0.0), candidates, 0, 0, 'shadow', (0.05, 0.0, 0.0, 1, 1, [100]))
    with open(os.path.join(tmp_path, 'decisions.jsonl')) as f:
        maps = [json.loads(line)['map'] for line in f]
    assert len(set(maps)) == 2
    assert np.load(os.path.join(tmp_path, maps[0]))['data'][0, 0] == 0
    assert np.load(os.path.join(tmp_path, maps[1]))['data'][0, 0] == 100
