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
