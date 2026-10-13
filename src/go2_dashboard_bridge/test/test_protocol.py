import json

import pytest

from go2_dashboard_bridge.protocol import SPORT_ACTIONS, normalize_objects, parse_voice_command


@pytest.mark.parametrize(
    ("phrase", "expected"),
    [
        ("stop now", {"type": "action", "action": "stop"}),
        ("recovery stand", {"type": "action", "action": "recover"}),
        ("lay down", {"type": "action", "action": "stand_down"}),
        ("pounce", {"type": "action", "action": "pounce"}),
        ("show me love", {"type": "action", "action": "finger_heart"}),
        ("dance two", {"type": "action", "action": "dance2"}),
        ("say I am here", {"type": "tts", "text": "i am here"}),
        ("walk forward", {"type": "movement_disabled"}),
        ("find my backpack", {"type": "unknown"}),
    ],
)
def test_parse_voice_command(phrase, expected):
    assert parse_voice_command(phrase) == expected


def test_added_dashboard_actions_use_unitree_sport_api_ids():
    assert SPORT_ACTIONS["stand_down"]["api_id"] == 1005
    assert SPORT_ACTIONS["pounce"]["api_id"] == 1032
    assert SPORT_ACTIONS["stand"]["followups"] == [
        {"api_id": 1002, "parameter": {}, "delay_sec": 2.0},
        {"api_id": 1027, "parameter": {"data": True}, "delay_sec": 2.4},
    ]


def test_normalize_objects_orders_by_distance():
    payload = json.dumps({
        "chair_1": {"x": 2, "y": 0},
        "person_2": {"x": 0.5, "y": -0.5},
    })
    objects = normalize_objects(payload)
    assert [item["id"] for item in objects] == ["person_2", "chair_1"]
    assert objects[0]["label"] == "person"
    assert objects[0]["distance"] == 0.71


def test_normalize_objects_skips_incomplete_entries():
    assert normalize_objects({"chair_1": {}, "person_1": {"x": 1, "y": 2}}) == [
        {"id": "person_1", "label": "person", "x": 1.0, "y": 2.0, "distance": 2.24, "bearing_deg": 63.4}
    ]
