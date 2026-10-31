"""ROS-dependent tests of the explorer's VLM ranking; run inside the Humble
container."""

import time
from types import SimpleNamespace

from go2_auto_mapper.explorer import FrontierExplorer
from go2_auto_mapper.vlm_ranking import (
    MOVE_CLEAR_M,
    Ranking,
    ViewMemory,
    suggestion_message,
)


class FakeClient:

    def __init__(self, result=None, busy=False):
        self.result = result
        self.is_busy = busy
        self.asked = []

    def busy(self):
        return self.is_busy

    def submit(self, goals, frames, facts=None):
        self.asked.append((goals, frames))
        self.facts = facts
        self.is_busy = True
        return True

    def take_result(self):
        result, self.result = self.result, None
        return result


def explorer_stub(mode, client):
    messages = []
    node = SimpleNamespace(
        vlm_mode=mode, vlm_client=client, vlm_weight=400.0, vlm_timeout=8.0,
        vlm_candidates=4, vlm_ranking=None, vlm_wait_started=None,
        vlm_retry_after=0.0, views=ViewMemory(bins=8), messages=messages,
        info_gain_cells_per_meter=100.0,
    )
    logger = SimpleNamespace(
        info=lambda m, **k: messages.append(m), warn=lambda m, **k: messages.append(m))
    node.get_logger = lambda: logger
    node.ask_vlm = lambda top, x, y: FrontierExplorer.ask_vlm(node, top, x, y)
    # A view facing each direction from the origin.
    for yaw in (0.0, 1.5708, 3.1416, -1.5708):
        node.views.remember(0.0, 0.0, yaw, f'frame@{yaw}', time.monotonic())
    return node


# (score, x, y, ...): lower score is better. The map score prefers the goal
# to the east; the VLM will prefer the one to the north.
POOL = [(-500.0, 3.0, 0.0, 10, True, True, False, 10.0, []),
        (-450.0, 0.0, 3.0, 10, True, True, False, 10.0, [])]


def scored(east, north):
    return Ranking([(3.0, 0.0), (0.0, 3.0)], [east, north], 'doorway north',
                   time.monotonic())


def test_asks_with_the_view_toward_each_goal():
    client = FakeClient()
    node = explorer_stub('shadow', client)
    assert FrontierExplorer.rank_with_vlm(node, POOL, 0.0, 0.0) == POOL
    goals, frames = client.asked[0]
    assert goals == [(3.0, 0.0), (0.0, 3.0)]
    assert frames == ['frame@0.0', 'frame@1.5708']


def test_asking_tells_the_model_what_the_map_says():
    client = FakeClient()
    FrontierExplorer.rank_with_vlm(explorer_stub('shadow', client), POOL, 0.0, 0.0)
    # POOL's first goal: score -500 for a 10 m walk at 100 cells a meter.
    assert '10.0 m away' in client.facts[0]
    assert '1500 unknown cells' in client.facts[0]


def test_shadow_mode_logs_but_keeps_the_map_choice():
    node = explorer_stub('shadow', FakeClient(result=scored(0.1, 0.9)))
    assert FrontierExplorer.rank_with_vlm(node, POOL, 0.0, 0.0) == POOL
    assert any('VLM differs' in m for m in node.messages)


def test_active_mode_lets_the_vlm_change_the_pick():
    node = explorer_stub('active', FakeClient(result=scored(0.1, 0.9)))
    ranked = FrontierExplorer.rank_with_vlm(node, POOL, 0.0, 0.0)
    best = min(ranked, key=lambda c: c[0])
    assert best[1:3] == (0.0, 3.0)   # -450 - 160 beats -500 + 160


def test_active_mode_waits_then_falls_back():
    client = FakeClient(busy=True)
    node = explorer_stub('active', client)
    assert FrontierExplorer.rank_with_vlm(node, POOL, 0.0, 0.0) is None   # wait
    node.vlm_wait_started -= 9.0   # longer than vlm_timeout
    assert FrontierExplorer.rank_with_vlm(node, POOL, 0.0, 0.0) == POOL


def test_a_failed_request_pauses_asking():
    failed = Ranking([(3.0, 0.0)], None, 'request failed: refused', time.monotonic())
    client = FakeClient(result=failed)
    node = explorer_stub('active', client)
    assert FrontierExplorer.rank_with_vlm(node, POOL, 0.0, 0.0) == POOL
    assert client.asked == []
    assert any('refused' in m for m in node.messages)


def suggestion_stub(text):
    node = explorer_stub('active', FakeClient())
    node.vlm_suggest_weight = 300.0
    node.vlm_suggestion = text
    return node


def test_a_clear_suggestion_changes_the_goal():
    # The map prefers east by 50; the VLM suggests north, with room that way.
    text = suggestion_message('left', 1.5708, MOVE_CLEAR_M + 1.0, time.time())
    node = suggestion_stub(text)
    ranked = FrontierExplorer.follow_suggestion(node, POOL, 0.0, 0.0)
    assert min(ranked, key=lambda c: c[0])[1:3] == (0.0, 3.0)
    assert any('suggestion changes' in m for m in node.messages)


def test_a_blocked_suggestion_leaves_the_goal_alone():
    text = suggestion_message('left', 1.5708, MOVE_CLEAR_M - 0.3, time.time())
    node = suggestion_stub(text)
    assert FrontierExplorer.follow_suggestion(node, POOL, 0.0, 0.0) == POOL


def test_no_suggestion_leaves_the_goal_alone():
    node = suggestion_stub(None)
    assert FrontierExplorer.follow_suggestion(node, POOL, 0.0, 0.0) == POOL


def test_no_views_means_map_score_alone():
    client = FakeClient()
    node = explorer_stub('active', client)
    node.views = ViewMemory(bins=8)
    assert FrontierExplorer.rank_with_vlm(node, POOL, 0.0, 0.0) == POOL
    assert client.asked == []
