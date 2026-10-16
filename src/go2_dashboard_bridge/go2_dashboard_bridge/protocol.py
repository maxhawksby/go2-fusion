"""Pure protocol helpers shared by the ROS bridge and offline tests."""

import json
import math
import re


SPORT_ACTIONS = {
    "stop": {"api_id": 1003, "label": "Request StopMove"},
    "stand": {
        "api_id": 1004,
        "label": "Stand up and enable handheld control",
        "followups": [
            {"api_id": 1002, "parameter": {}, "delay_sec": 2.0},
            {"api_id": 1027, "parameter": {"data": True}, "delay_sec": 2.4},
        ],
    },
    "stand_down": {"api_id": 1005, "label": "Stand down"},
    "recover": {"api_id": 1006, "label": "Recovery stand"},
    "sit": {"api_id": 1009, "label": "Sit"},
    "hello": {"api_id": 1016, "label": "Hello"},
    "stretch": {"api_id": 1017, "label": "Stretch"},
    "dance1": {"api_id": 1022, "label": "Dance 1"},
    "dance2": {"api_id": 1023, "label": "Dance 2"},
    "pounce": {"api_id": 1032, "label": "Front pounce"},
    "finger_heart": {"api_id": 1036, "label": "Show love"},
}


def parse_voice_command(text):
    """Map deterministic voice phrases to the stable dashboard API."""
    cleaned = " ".join(str(text).strip().lower().split())
    if not cleaned:
        return {"type": "unknown"}

    # The wake phrase is optional. Removing it here keeps the command patterns
    # focused on intent and accepts common punctuation after "Ben Ben".
    cleaned = re.sub(r"^hey\s+ben\s+ben\b[\s,.:;-]*", "", cleaned).strip()

    say_match = re.match(r"^(?:say|speak)\s+(.+)$", cleaned)
    if say_match:
        return {"type": "tts", "text": say_match.group(1)}

    patterns = (
        (r"\b(?:stop|halt|freeze)\b", "stop"),
        (r"\b(?:recovery stand|recover)\b", "recover"),
        (r"\b(?:stand down|lay down|lie down)\b", "stand_down"),
        (r"\bstand(?: up)?\b", "stand"),
        (r"\bsit(?: down)?\b", "sit"),
        (r"\b(?:hello|wave)\b", "hello"),
        (r"\bstretch\b", "stretch"),
        (r"\b(?:front )?pounce\b", "pounce"),
        (r"\bdance (?:two|2)\b", "dance2"),
        (r"\bdance(?: one| 1)?\b", "dance1"),
        (r"\b(?:show me (?:how much you )?love(?: me)?|show love|finger heart)\b", "finger_heart"),
    )
    for pattern, action in patterns:
        if re.search(pattern, cleaned):
            return {"type": "action", "action": action}

    if re.search(r"\b(?:forward|backwards?|turn left|turn right|walk|move)\b", cleaned):
        return {"type": "movement_disabled"}
    return {"type": "unknown"}


def normalize_objects(payload):
    """Convert the detector's JSON dictionary into a frontend-stable list."""
    raw = json.loads(payload) if isinstance(payload, str) else payload
    if not isinstance(raw, dict):
        raise ValueError("Object payload must be a JSON object")

    objects = []
    for object_id, position in raw.items():
        if not isinstance(position, dict):
            continue
        try:
            x = float(position["x"])
            y = float(position["y"])
        except (KeyError, TypeError, ValueError):
            continue
        objects.append({
            "id": str(object_id),
            "label": re.sub(r"_\d+$", "", str(object_id)).replace("_", " "),
            "x": round(x, 2),
            "y": round(y, 2),
            "distance": round(math.hypot(x, y), 2),
            "bearing_deg": round(math.degrees(math.atan2(y, x)), 1),
        })
    return sorted(objects, key=lambda item: (item["distance"], item["id"]))
