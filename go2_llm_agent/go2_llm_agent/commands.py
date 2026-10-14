"""Small, provider-independent action contract. No ROS or model side effects."""

from dataclasses import dataclass, replace
import json
import math
import re

MAX_DURATION_S = 5.0
MAX_PLAN_DURATION_S = 45.0
DEFAULT_WALK_DURATION_S = 1.0
DEFAULT_TURN_ANGLE_DEG = 90.0
DEFAULT_SPIN_ANGLE_DEG = 360.0
MAX_STEPS = 4
LINEAR_SPEED_MPS = 0.2
ANGULAR_SPEED_RAD_S = 0.5

PLAN_SCHEMA = {
    "type": "object",
    "additionalProperties": False,
    "required": ["status", "reason", "steps"],
    "properties": {
        "status": {"type": "string", "enum": ["execute", "clarify", "unsupported", "chat"]},
        "reason": {"type": "string"},
        "steps": {
            "type": "array",
            "items": {
                "type": "object",
                "additionalProperties": False,
                "required": ["action", "direction", "quantity", "unit"],
                "properties": {
                    "action": {"type": "string", "enum": ["walk", "turn", "spin", "stop"]},
                    "direction": {"type": "string", "enum": ["forward", "backward", "left", "right", "none"]},
                    "quantity": {"type": ["number", "null"]},
                    "unit": {"type": "string", "enum": ["seconds", "degrees", "default"]},
                },
            },
        },
    },
}


class InvalidPlan(ValueError):
    pass


@dataclass(frozen=True)
class Step:
    action: str
    direction: str
    duration_s: float
    angle_deg: float | None = None

    def time_limit(self):
        if self.angle_deg is not None:
            return math.radians(self.angle_deg) / ANGULAR_SPEED_RAD_S * 2 + 3
        return self.duration_s

    def velocities(self):
        if self.action == "walk":
            return (LINEAR_SPEED_MPS if self.direction == "forward" else -LINEAR_SPEED_MPS, 0.0)
        if self.action in {"turn", "spin"}:
            return (0.0, ANGULAR_SPEED_RAD_S if self.direction == "left" else -ANGULAR_SPEED_RAD_S)
        return (0.0, 0.0)


@dataclass(frozen=True)
class Plan:
    status: str
    reason: str
    steps: tuple

    def as_dict(self):
        return {"status": self.status, "reason": self.reason,
                "steps": [{"action": s.action, "direction": s.direction,
                           "duration_s": s.duration_s, "angle_deg": s.angle_deg} for s in self.steps]}


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise InvalidPlan("Duplicate JSON field: " + key)
        result[key] = value
    return result


def _invalid_number(value):
    raise InvalidPlan("Non-finite JSON number: " + value)


def validate_plan(content):
    """Validate again locally; schema-constrained generation is not trusted."""
    if not isinstance(content, str) or len(content) > 16384:
        raise InvalidPlan("Model response must be a small JSON object.")
    try:
        obj = json.loads(content, object_pairs_hook=_unique_object, parse_constant=_invalid_number)
    except (ValueError, RecursionError) as exc:
        raise InvalidPlan("Invalid model JSON: " + str(exc)) from exc
    if not isinstance(obj, dict) or set(obj) != {"status", "reason", "steps"}:
        raise InvalidPlan("Expected exactly status, reason and steps.")
    status, reason, raw_steps = obj["status"], obj["reason"], obj["steps"]
    if not isinstance(status, str) or status not in {"execute", "clarify", "unsupported", "chat"}:
        raise InvalidPlan("Unknown plan status.")
    if not isinstance(reason, str) or not reason.strip() or len(reason) > 1000:
        raise InvalidPlan("Expected a short explanation.")
    if not isinstance(raw_steps, list) or len(raw_steps) > MAX_STEPS:
        raise InvalidPlan("At most four action steps are allowed.")
    if status != "execute" and raw_steps:
        raise InvalidPlan("Non-executable responses must have no steps.")
    if status == "execute" and not raw_steps:
        raise InvalidPlan("Executable plans need an action.")
    steps = []
    for raw in raw_steps:
        if not isinstance(raw, dict) or set(raw) != {"action", "direction", "quantity", "unit"}:
            raise InvalidPlan("Unexpected action fields.")
        action, direction, quantity, unit = (raw[k] for k in ("action", "direction", "quantity", "unit"))
        if not all(isinstance(v, str) for v in (action, direction, unit)):
            raise InvalidPlan("Action, direction and unit must be strings.")
        directions = {"walk": {"forward", "backward", "none"}, "turn": {"left", "right", "none"},
                      "spin": {"left", "right", "none"}, "stop": {"none"}}
        if action not in directions or direction not in directions[action]:
            raise InvalidPlan("Action and direction do not match.")
        if unit == "default":
            if quantity is not None:
                raise InvalidPlan("Default actions must have a null quantity.")
            if action == "walk":
                unit, quantity = "seconds", DEFAULT_WALK_DURATION_S
            elif action == "stop":
                unit, quantity = "seconds", 0.0
            else:
                unit = "degrees"
                quantity = DEFAULT_SPIN_ANGLE_DEG if action == "spin" else DEFAULT_TURN_ANGLE_DEG
        if type(quantity) not in (int, float):
            raise InvalidPlan("Quantity must be a number, not text or a boolean.")
        try:
            quantity = float(quantity)
        except (ValueError, OverflowError) as exc:
            raise InvalidPlan("Quantity is out of range.") from exc
        if not math.isfinite(quantity):
            raise InvalidPlan("Quantity must be finite.")
        angle = None
        if action == "stop":
            if unit != "seconds" or quantity != 0:
                raise InvalidPlan("Stop must have zero seconds.")
        elif unit == "seconds":
            if not 0 < quantity <= MAX_DURATION_S:
                return Plan("unsupported", "Timed movement must be greater than zero and at most five seconds.", ())
        elif unit == "degrees" and action in {"turn", "spin"}:
            if not 0 < quantity <= 360:
                return Plan("unsupported", "Turn angle must be greater than zero and at most 360 degrees.", ())
            angle, quantity = quantity, 0.0
        else:
            raise InvalidPlan("Unsupported movement unit.")
        if action == "spin" and direction == "none":
            direction = "left"
        steps.append(Step(action, direction, quantity, angle))
    if sum(s.time_limit() for s in steps) > MAX_PLAN_DURATION_S:
        return Plan("unsupported", "Plan exceeds the 45-second execution budget, including turn timeouts.", ())
    if any(s.direction == "none" and s.action != "stop" for s in steps):
        return Plan("clarify", "Should I move forward/backward or turn left/right?", ())
    return Plan(status, reason.strip(), tuple(steps))


def is_stop(text):
    normalized = " ".join(text.strip().lower().rstrip(".!?").split())
    return normalized in {"stop", "stop now", "stop please", "please stop", "halt", "cancel", "emergency stop",
                          "stop moving", "stop walking", "stop turning", "stop the robot", "stop the dog",
                          "stop oak", "please stop moving", "cancel movement", "cancel command"}


_WORDS = {"zero": 0, "one": 1, "two": 2, "three": 3, "four": 4, "five": 5,
          "six": 6, "seven": 7, "eight": 8, "nine": 9, "ten": 10, "a": 1, "an": 1,
          "half": 0.5, "half a": 0.5, "quarter": 0.25, "quarter of a": 0.25, "a quarter": 0.25}
_AMOUNT = (r"[+-]?(?:\d+(?:\.\d+)?|\.\d+)"
           r"|(?:zero|one|two|three|four|five|six|seven|eight|nine|ten)(?: and (?:a )?(?:half|quarter))?"
           r"|half(?: a)?|quarter(?: of a)?|a quarter|an|a")
_TIMES = re.compile(r"(?<![\w.])(?P<amount>(?:(?:minus|negative)\s+)?(?:" + _AMOUNT +
                    r"))\s*(?P<unit>milliseconds?|msecs?|ms|seconds?|secs?|s)\b", re.I)
_DIRECTIONS = {
    "forward": r"\b(?:forwards?|ahead)\b",
    "backward": r"\b(?:backwards?|reverse|back)\b",
    "left": r"\b(?:left|counter-?clockwise|anti-?clockwise|ccw)\b",
    "right": r"\bright\b|(?<![\w-])(?:clockwise|cw)\b",
}
_QUALIFIED_REQUEST = re.compile(r"\b(?:not|no|never|avoid|if|unless|until|when)\b|\bdon['’]t\b", re.I)


def explicit_durations(text):
    """Conservative evidence check, not an alternative general NLP parser."""
    values = []
    for match in _TIMES.finditer(text):
        amount = " ".join(match["amount"].lower().split())
        sign = -1 if amount.startswith(("minus ", "negative ")) else 1
        if sign < 0:
            amount = amount.split(" ", 1)[1]
        if " and " in amount:
            whole, fraction = amount.split(" and ", 1)
            value = _WORDS[whole] + (0.5 if "half" in fraction else 0.25)
        elif amount in _WORDS:
            value = _WORDS[amount]
        else:
            value = float(amount)
        if match["unit"].lower().startswith(("m", "ms")):
            value /= 1000
        values.append(sign * value)
    return values


_ANGLES = re.compile(r"(?<![\w.])(?P<amount>(?:(?:minus|negative)\s+)?(?:" + _AMOUNT + r"))\s*(?:degrees?|deg|°)(?!\w)", re.I)
_ACTIONS = re.compile(r"\b(?P<walk>walk|move|go|back\s+up|reverse)\b|\b(?P<turn>turn|rotate)\b|\b(?P<spin>spin)\b|\b(?P<stop>stop|halt)\b", re.I)
_STEP_COUNTS = re.compile(r"\b(?:steps?|paces?)\b", re.I)


def validate_request(plan, text):
    """Check explicit quantities per action; only documented defaults may fill gaps.

    This conservative vocabulary check does not prove arbitrary English semantics.
    """
    if plan.status != "execute":
        return plan
    if _STEP_COUNTS.search(text):
        return Plan("unsupported", "Counting footsteps is not implemented. Specify walking time in seconds.", ())
    if any(s.action != "stop" for s in plan.steps) and _QUALIFIED_REQUEST.search(text):
        return Plan("unsupported", "Negated or conditional movements are not supported. Give a direct movement request.", ())
    actions = list(_ACTIONS.finditer(text))
    if len(actions) != len(plan.steps):
        return Plan("clarify", "Please name each movement, such as walk forward then turn left.", ())
    normalized = []
    for index, (source, step) in enumerate(zip(actions, plan.steps)):
        kind = source.lastgroup
        clause = text[source.start():actions[index + 1].start() if index + 1 < len(actions) else len(text)]
        if kind in {"turn", "spin"} and step.action in {"turn", "spin"}:
            step = replace(step, action=kind)  # Same rotational motion; verify its quantity below.
        elif kind != step.action:
            raise InvalidPlan("The model changed an action or its order. Please rephrase; no actions ran.")
        if step.action == "stop":
            normalized.append(step)
            continue
        mentioned = sorted((m.start(), direction) for direction, pattern in _DIRECTIONS.items()
                           for m in re.finditer(pattern, clause, re.I))
        directions = [direction for _, direction in mentioned]
        if not directions:
            if kind != "spin":
                return Plan("clarify", "Please specify each walking or turning direction.", ())
            directions = ["left"]  # Documented full-spin default, owned by code.
        if directions != [step.direction]:
            raise InvalidPlan("The model changed a direction. Please rephrase; no actions ran.")
        durations = explicit_durations(clause)
        # Reuse the numeric word/unit parser, including fractions and negative amounts.
        angles = explicit_durations(_ANGLES.sub(lambda m: m["amount"] + " seconds", clause)) if _ANGLES.search(clause) else []
        if durations and angles:
            raise InvalidPlan("Use seconds or degrees for a turn, not both.")
        if len(durations) > 1 or len(angles) > 1:
            raise InvalidPlan("Use one quantity per movement.")
        if durations:
            expected_duration, expected_angle = durations[0], None
        elif angles:
            expected_duration, expected_angle = 0.0, angles[0]
        else:
            if re.search(r"\b(?:for|by|meters?|metres?|cm|feet|inches|forever|continuously|twice)\b", clause, re.I):
                return Plan("clarify", "I could not verify that quantity. Use seconds or degrees.", ())
            expected_duration = DEFAULT_WALK_DURATION_S if kind == "walk" else 0.0
            expected_angle = None if kind == "walk" else (DEFAULT_SPIN_ANGLE_DEG if kind == "spin" else DEFAULT_TURN_ANGLE_DEG)
        if step.angle_deg != expected_angle or not math.isclose(step.duration_s, expected_duration, rel_tol=1e-9, abs_tol=1e-9):
            raise InvalidPlan("The model changed a quantity or the documented default. Please rephrase; no actions ran.")
        normalized.append(step)
    return Plan(plan.status, plan.reason, tuple(normalized))
