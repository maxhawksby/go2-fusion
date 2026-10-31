import json
import math
import time
import urllib.error

from go2_auto_mapper.vlm_ranking import (
    NEUTRAL_SCORE,
    Ranking,
    ViewMemory,
    VlmClient,
    MOVE_CLEAR_M,
    build_describe_request,
    clear_distance,
    lidar_sentence,
    describe_move,
    parse_suggestion,
    suggestion_bonus,
    suggestion_message,
    usable_heading,
    build_request,
    heading_bin,
    match_score,
    merge_scores,
    parse_reply,
    vlm_bonus,
)


def wait_for_result(client, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = client.take_result()
        if result is not None:
            return result
        time.sleep(0.01)
    raise AssertionError('no result from the background thread')


def test_heading_bins_split_a_full_turn():
    assert heading_bin(0.0, 8) == 0
    assert heading_bin(math.pi / 2, 8) == 2
    assert heading_bin(-math.pi / 2, 8) == 6   # negative yaw wraps around
    assert heading_bin(2.0 * math.pi, 8) == 0


def test_view_toward_picks_the_closest_heading():
    views = ViewMemory(bins=8)
    views.remember(0.0, 0.0, 0.0, 'east', now=10.0)
    views.remember(0.0, 0.0, math.pi / 2, 'north', now=10.0)
    frame = views.view_toward(0.0, 0.0, math.radians(80), 11.0, 1.0, math.radians(35), 60.0)
    assert frame == 'north'


def test_view_toward_rejects_far_old_or_off_angle_views():
    views = ViewMemory(bins=8)
    views.remember(0.0, 0.0, 0.0, 'east', now=10.0)
    args = (math.radians(35), 60.0)
    assert views.view_toward(5.0, 0.0, 0.0, 11.0, 1.0, *args) is None   # robot moved
    assert views.view_toward(0.0, 0.0, 0.0, 100.0, 1.0, *args) is None  # too old
    assert views.view_toward(0.0, 0.0, math.pi, 11.0, 1.0, *args) is None  # wrong way


def test_newer_view_replaces_older_in_same_heading():
    views = ViewMemory(bins=8)
    views.remember(0.0, 0.0, 0.0, 'old', now=1.0)
    views.remember(0.0, 0.0, 0.1, 'new', now=2.0)
    assert views.view_toward(0.0, 0.0, 0.0, 3.0, 1.0, 0.5, 60.0) == 'new'


def test_request_carries_one_image_per_candidate():
    payload = build_request('qwen', [b'a', b'b'])
    content = payload['messages'][0]['content']
    images = [part for part in content if part['type'] == 'image_url']
    assert payload['model'] == 'qwen'
    assert len(images) == 2
    assert images[0]['image_url']['url'].startswith('data:image/jpeg;base64,')
    assert '2 images' in content[0]['text']


def test_request_can_carry_what_the_map_says():
    content = build_request('qwen', [b'a', b'b'], ['near', 'far'])['messages'][0]['content']
    texts = [part['text'] for part in content if part['type'] == 'text']
    assert texts[1] == 'Image 1: near'
    assert texts[2] == 'Image 2: far'


def test_facts_stay_with_their_pictures_when_a_view_is_missing():
    sent = {}

    def fake_post(url, payload, timeout, api_key):
        sent['content'] = payload['messages'][0]['content']
        return '{"scores": [0.8, 0.3]}'

    client = VlmClient('http://x', 'm', 1.0, encode=lambda f: f, post=fake_post)
    client.submit([(0, 0), (1, 0), (2, 0)], [b'a', None, b'c'], ['A', 'B', 'C'])
    wait_for_result(client)
    texts = [part['text'] for part in sent['content'] if part['type'] == 'text']
    assert texts[1:] == ['Image 1: A', 'Image 2: C']


def test_lidar_sentence_gives_each_direction():
    sentence = lidar_sentence([1.0, 2.0, 3.0, float('inf')], 0.0, math.pi / 2)
    assert sentence == ('Lidar distances now: forward 1.0 m, left 2.0 m, '
                        'right no reading, behind 3.0 m.')


def test_describe_request_tells_the_model_what_the_lidar_sees():
    payload = build_describe_request('qwen', b'a', 'Lidar distances now: forward 1.0 m.')
    assert 'forward 1.0 m' in payload['messages'][0]['content'][0]['text']
    assert payload['response_format'] == {'type': 'json_object'}


def test_describe_request_carries_the_picture():
    content = build_describe_request('qwen', b'a')['messages'][0]['content']
    assert content[0]['type'] == 'text'
    assert content[1]['image_url']['url'].startswith('data:image/jpeg;base64,')


def test_suggestion_reads_a_known_move():
    reply = 'Sure: {"description": "A hallway.", "move": "Turn Around", "reason": "Dead end."}'
    assert parse_suggestion(reply) == ('A hallway.', 'turn_around', 'Dead end.')


def test_suggestion_without_a_known_move_keeps_what_it_said():
    assert parse_suggestion('just prose')[:2] == ('just prose', None)
    assert parse_suggestion('{"description": "x", "move": "fly"}')[1] is None
    assert parse_suggestion(None) == ('', None, '')


def test_clear_distance_looks_only_that_way():
    # Four readings: ahead, left, behind, right.
    ranges = [1.0, 2.0, 3.0, 4.0]
    step = math.pi / 2
    assert clear_distance(ranges, 0.0, step, 0.0) == 1.0
    assert clear_distance(ranges, 0.0, step, math.pi / 2) == 2.0
    assert clear_distance(ranges, 0.0, step, -math.pi / 2) == 4.0
    assert clear_distance(ranges, 0.0, step, math.pi) == 3.0   # wraps around


def test_clear_distance_skips_bad_readings():
    assert clear_distance([float('inf'), float('nan'), 0.0], 0.0, 0.01, 0.0) is None


def test_describe_move_flags_a_blocked_way():
    assert 'BLOCKED' in describe_move('forward', MOVE_CLEAR_M - 0.1, True)
    assert 'clear' in describe_move('left', MOVE_CLEAR_M + 1.0, True)
    assert 'not checked' in describe_move('left', None, False)
    assert 'not checked' in describe_move('left', None, True)
    assert describe_move('stop', None, True) == ''
    assert describe_move(None, None, True) == ''


def test_a_recent_clear_suggestion_gives_its_heading():
    text = suggestion_message('left', 1.5, MOVE_CLEAR_M + 1.0, now=100.0)
    assert usable_heading(text, 105.0) == 1.5


def test_a_blocked_old_or_directionless_suggestion_is_ignored():
    clear = MOVE_CLEAR_M + 1.0
    blocked = suggestion_message('left', 1.5, MOVE_CLEAR_M - 0.1, 100.0)
    assert usable_heading(blocked, 101.0) is None
    assert usable_heading(suggestion_message('left', 1.5, None, 100.0), 101.0) is None
    assert usable_heading(suggestion_message('left', 1.5, clear, 100.0), 200.0) is None
    assert usable_heading(suggestion_message('stop', None, clear, 100.0), 101.0) is None
    assert usable_heading(None, 101.0) is None
    assert usable_heading('not json', 101.0) is None


def test_bonus_goes_only_to_goals_that_way():
    assert suggestion_bonus(0.2, 0.0, 300.0) == 300.0
    assert suggestion_bonus(-0.2, 0.0, 300.0) == 300.0
    assert suggestion_bonus(math.pi, 0.0, 300.0) == 0.0


def test_parse_reply_reads_json_inside_prose():
    text = 'Sure!\n```json\n{"scores": [0.9, 0.1], "reason": "door on the left"}\n```'
    scores, reason = parse_reply(text, 2)
    assert scores == [0.9, 0.1]
    assert reason == 'door on the left'


def test_parse_reply_clamps_scores():
    scores, _ = parse_reply('{"scores": [1.7, -2]}', 2)
    assert scores == [1.0, 0.0]


def test_parse_reply_rejects_bad_replies():
    assert parse_reply('no json here', 2)[0] is None
    assert parse_reply('{"answer": 1}', 2)[0] is None
    assert parse_reply('{"scores": ["high", 0.2]}', 2)[0] is None
    assert parse_reply('{"scores": [0.5]}', 2)[0] is None   # wrong count
    assert parse_reply(None, 1)[0] is None


def test_candidates_without_a_view_get_the_neutral_score():
    assert merge_scores([True, False, True], [0.9, 0.2]) == [0.9, NEUTRAL_SCORE, 0.2]


def test_bonus_is_zero_for_a_neutral_score():
    assert vlm_bonus(NEUTRAL_SCORE, 400.0) == 0.0
    assert vlm_bonus(1.0, 400.0) == 200.0
    assert vlm_bonus(0.0, 400.0) == -200.0


def test_match_score_finds_goals_that_moved_a_little():
    ranking = Ranking([(0.0, 0.0), (3.0, 0.0)], [0.9, 0.2], '', 0.0)
    assert match_score(ranking, 0.2, 0.1, 0.5) == 0.9
    assert match_score(ranking, 3.1, 0.0, 0.5) == 0.2
    assert match_score(ranking, 1.5, 0.0, 0.5) is None
    assert match_score(Ranking([(0.0, 0.0)], None, 'failed', 0.0), 0.0, 0.0, 0.5) is None
    assert match_score(None, 0.0, 0.0, 0.5) is None


def test_client_scores_in_the_background():
    sent = {}

    def fake_post(url, payload, timeout, api_key):
        sent['images'] = sum(
            part['type'] == 'image_url' for part in payload['messages'][0]['content']
        )
        return json.dumps({'scores': [0.8, 0.3], 'reason': 'hallway'})

    client = VlmClient('http://x', 'm', 1.0, encode=lambda f: f, post=fake_post)
    assert client.submit([(0, 0), (1, 0), (2, 0)], [b'a', None, b'c'])
    result = wait_for_result(client)
    assert sent['images'] == 2   # the candidate with no view is not sent
    assert result.scores == [0.8, NEUTRAL_SCORE, 0.3]
    assert result.reason == 'hallway'
    assert client.take_result() is None   # a result is handed out once


def test_client_reports_a_down_server_instead_of_raising():
    def failing_post(url, payload, timeout, api_key):
        raise urllib.error.URLError('connection refused')

    client = VlmClient('http://x', 'm', 1.0, encode=lambda f: f, post=failing_post)
    client.submit([(0, 0)], [b'a'])
    result = wait_for_result(client)
    assert result.scores is None
    assert 'connection refused' in result.reason


def test_client_runs_one_request_at_a_time():
    release = []

    def slow_post(url, payload, timeout, api_key):
        while not release:
            time.sleep(0.01)
        return '{"scores": [0.5]}'

    client = VlmClient('http://x', 'm', 1.0, encode=lambda f: f, post=slow_post)
    assert client.submit([(0, 0)], [b'a'])
    assert client.busy()
    assert not client.submit([(0, 0)], [b'a'])
    release.append(True)
    wait_for_result(client)
