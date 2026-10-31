"""Asking a vision-language model (VLM) which frontier looks most worth
exploring, from the camera's view toward each one.

The model runs on a GPU server behind an OpenAI-compatible chat API (vLLM
and Ollama both serve one), usually reached through an SSH tunnel. Nothing
here needs ROS, so it can be tested with hand-made data.

The VLM only re-ranks goals the explorer already found free and clear of
walls: lidar, SLAM and Nav2 still decide where the robot can walk.
"""

import base64
import json
import math
import re
import threading
import time
import urllib.error
import urllib.request
from collections import namedtuple

from go2_auto_mapper.obstacle_sensing import angle_difference

# The score a candidate gets when the VLM can't judge it (no camera view
# toward it): neither a bonus nor a penalty.
NEUTRAL_SCORE = 0.5

PROMPT = (
    'You are helping a robot dog, 0.7 m long and 0.3 m wide, map an unfamiliar '
    'indoor building. Each of the {n} images below is its camera view toward a '
    'different unexplored area, with what its map says about that area. The map '
    'already knows the distances and walls. Use the picture for what the map '
    'cannot tell: rate from 0.0 to 1.0 how likely walking that way reveals a lot '
    'of new space. An open doorway, a hallway or an open room score high; a wall, '
    'a closed door, a dead end, or furniture score low. Chair legs and gaps under '
    'desks are not paths. If you cannot tell, answer 0.5 and do not invent a '
    'doorway. Reply with JSON only, in this form, with exactly {n} scores: '
    '{{"scores": [one number per image, in order], "reason": "one short sentence"}}'
)

DESCRIBE_PROMPT = (
    'You are the camera of a robot dog, 0.7 m long and 0.3 m wide, exploring a '
    'building to map it. Describe what you see, then suggest its next move, toward '
    'a doorway, hallway or open room it could map. Its lidar measures distances you '
    'cannot judge from a picture, and it only knows about what is in front of the '
    'camera for forward. {lidar} Never suggest a direction the lidar says is under '
    '{clear:.1f} m. Chair legs and gaps under desks are not paths. Do not invent a '
    'doorway you cannot see. Reply with JSON only, in this form: '
    '{{"description": "2 short sentences: the room, objects, people, and any open '
    'space or doorway", "move": "forward, left, right, turn_around or stop", '
    '"reason": "one short sentence"}}'
)

# Moves the model may suggest, with the direction each would walk in (radians,
# left positive) and how to say it. Stop has no direction to check.
MOVES = {
    'forward': (0.0, 'go forward'),
    'left': (math.pi / 2, 'turn left'),
    'right': (-math.pi / 2, 'turn right'),
    'turn_around': (math.pi, 'turn around'),
    'stop': (None, 'stop'),
}
# The lidar counts a direction as blocked closer than this to base_link: the
# nose is 0.36 m ahead of it, plus room to stop.
MOVE_CLEAR_M = 0.8
# A suggestion older than this describes where the robot used to be.
SUGGESTION_MAX_AGE_SEC = 15.0
# A goal this close to the suggested heading counts as following it.
SUGGESTION_CONE_RAD = math.radians(45.0)
# How wide a slice of the scan a direction covers, each side.
MOVE_CONE_RAD = math.radians(20.0)

# One finished request: the goals asked about as (x, y), the score for
# each (None if the request failed), the model's one-line reason or the
# error, and time.monotonic() when it finished.
Ranking = namedtuple('Ranking', 'goals scores reason finished_at')


def heading_bin(yaw, bins):
    """Which of `bins` equal slices of a full turn the heading falls in."""
    turn = (yaw % (2.0 * math.pi)) / (2.0 * math.pi)
    return int(turn * bins) % bins


class ViewMemory:
    """The latest camera frame per heading, with where the robot stood.

    The robot turns a full circle at the start and after every goal, so
    this fills with a view in every direction from where it now stands.
    Frames are kept as-is; they are only encoded when a request is sent.
    """

    def __init__(self, bins=8):
        self.bins = bins
        self.views = {}

    def remember(self, x, y, yaw, frame, now):
        self.views[heading_bin(yaw, self.bins)] = (x, y, yaw, frame, now)

    def view_toward(self, x, y, bearing, now, max_offset_m, max_angle, max_age):
        """The frame best facing `bearing` from near (x, y), or None.

        A view only counts if it was taken within `max_offset_m` of (x, y),
        at most `max_age` seconds ago, and within `max_angle` radians of
        the bearing; otherwise it shows a different place or direction.
        """
        best = None
        for vx, vy, yaw, frame, taken in self.views.values():
            if now - taken > max_age:
                continue
            if math.hypot(vx - x, vy - y) > max_offset_m:
                continue
            off = angle_difference(yaw, bearing)
            if off <= max_angle and (best is None or off < best[0]):
                best = (off, frame)
        return None if best is None else best[1]


def build_request(model, jpegs, facts=None):
    """Chat-completions payload asking the model to score each image. facts,
    if given, is one line per image of what the map says about that area."""
    content = [{'type': 'text', 'text': PROMPT.format(n=len(jpegs))}]
    for i, jpeg in enumerate(jpegs, start=1):
        fact = f' {facts[i - 1]}' if facts else ''
        content.append({'type': 'text', 'text': f'Image {i}:{fact}'})
        content.append({
            'type': 'image_url',
            'image_url': {
                'url': 'data:image/jpeg;base64,' + base64.b64encode(jpeg).decode('ascii')
            },
        })
    return {
        'model': model,
        'messages': [{'role': 'user', 'content': content}],
        'temperature': 0.0,
        'max_tokens': 200,
        'response_format': {'type': 'json_object'},
    }


def lidar_sentence(ranges, angle_min, angle_increment):
    """The lidar's distance each way, in words, for the prompt."""
    parts = []
    for name, bearing in (('forward', 0.0), ('left', math.pi / 2), ('right', -math.pi / 2),
                          ('behind', math.pi)):
        nearest = clear_distance(ranges, angle_min, angle_increment, bearing)
        parts.append(f'{name} no reading' if nearest is None else f'{name} {nearest:.1f} m')
    return 'Lidar distances now: ' + ', '.join(parts) + '.'


def build_describe_request(model, jpeg, lidar='The lidar is not available now.'):
    """Chat-completions payload asking the model to describe one image."""
    prompt = DESCRIBE_PROMPT.format(lidar=lidar, clear=MOVE_CLEAR_M)
    url = 'data:image/jpeg;base64,' + base64.b64encode(jpeg).decode('ascii')
    return {
        'model': model,
        'messages': [{'role': 'user', 'content': [
            {'type': 'text', 'text': prompt},
            {'type': 'image_url', 'image_url': {'url': url}},
        ]}],
        'temperature': 0.0,
        'max_tokens': 160,
        'response_format': {'type': 'json_object'},
    }


def parse_suggestion(text):
    """(description, move, reason) from the model's reply. move is None if it
    gave no known move; then description is whatever it said."""
    match = re.search(r'\{.*\}', text or '', re.DOTALL)
    try:
        data = json.loads(match.group(0))
        move = str(data.get('move', '')).strip().lower().replace(' ', '_')
        return (str(data.get('description', '')),
                move if move in MOVES else None,
                str(data.get('reason', '')))
    except (AttributeError, ValueError):
        return text or '', None, ''


def clear_distance(ranges, angle_min, angle_increment, bearing, cone=MOVE_CONE_RAD):
    """Nearest lidar reading within cone of bearing (radians, 0 ahead), or None
    if the scan has no valid reading there."""
    nearest = None
    for i, reading in enumerate(ranges):
        if not math.isfinite(reading) or reading <= 0.0:
            continue
        off = (angle_min + i * angle_increment - bearing + math.pi) % (2.0 * math.pi) - math.pi
        if abs(off) <= cone and (nearest is None or reading < nearest):
            nearest = reading
    return nearest


def describe_move(move, nearest, have_scan):
    """One line saying what the lidar thinks of a suggested move."""
    if move is None or MOVES[move][0] is None:
        return ''
    if not have_scan:
        return 'Lidar: no recent scan, not checked.'
    if nearest is None:
        return 'Lidar: no reading that way, not checked.'
    if nearest < MOVE_CLEAR_M:
        return f'Lidar: BLOCKED, something {nearest:.1f} m that way.'
    return f'Lidar: clear for {nearest:.1f} m.'


def suggestion_message(move, heading, nearest, now):
    """The /vlm/suggestion JSON. heading is the map-frame direction to walk (None
    if unknown); clear only when the lidar saw room that way."""
    return json.dumps({
        'move': move,
        'heading': heading,
        'clear': nearest is not None and nearest >= MOVE_CLEAR_M,
        'time': now,
    })


def usable_heading(text, now, max_age=SUGGESTION_MAX_AGE_SEC):
    """The map-frame heading from a /vlm/suggestion message, or None unless it
    is recent, has a direction, and the lidar saw room that way."""
    try:
        data = json.loads(text)
        if now - float(data['time']) > max_age or data.get('clear') is not True:
            return None
        return None if data.get('heading') is None else float(data['heading'])
    except (TypeError, ValueError, KeyError):
        return None


def suggestion_bonus(goal_bearing, heading, weight, cone=SUGGESTION_CONE_RAD):
    """How much better (higher) a goal scores for lying the way the VLM suggests."""
    return weight if abs(angle_difference(goal_bearing, heading)) < cone else 0.0


def parse_reply(text, count):
    """(scores, reason) from the model's reply, or (None, why) if unusable.

    Models often wrap JSON in prose or a code fence, so the first {...}
    block is read. Scores are clamped to 0..1, and there must be exactly
    one per image.
    """
    match = re.search(r'\{.*\}', text or '', re.DOTALL)
    if match is None:
        return None, 'reply had no JSON'
    try:
        data = json.loads(match.group(0))
        scores = [min(1.0, max(0.0, float(s))) for s in data['scores']]
    except (ValueError, KeyError, TypeError):
        return None, 'reply JSON had no usable "scores" list'
    if len(scores) != count:
        return None, f'expected {count} scores, got {len(scores)}'
    return scores, str(data.get('reason', ''))


def merge_scores(has_view, scores):
    """One score per candidate: the model's, in order, for those with a
    view, and NEUTRAL_SCORE for those without (the model never saw them)."""
    model_scores = iter(scores)
    return [next(model_scores) if seen else NEUTRAL_SCORE for seen in has_view]


def vlm_bonus(score, weight):
    """Cells of information gain a VLM score is worth: +weight/2 for a
    sure yes, -weight/2 for a sure no, 0 for a shrug."""
    return weight * (score - NEUTRAL_SCORE)


def match_score(ranking, x, y, radius):
    """The ranking's score for the goal within `radius` of (x, y), or None.

    Goals move a little each time SLAM updates the map, so they are
    matched by distance rather than exactly.
    """
    if ranking is None or ranking.scores is None:
        return None
    best = None
    for (gx, gy), score in zip(ranking.goals, ranking.scores):
        d = math.hypot(gx - x, gy - y)
        if d <= radius and (best is None or d < best[0]):
            best = (d, score)
    return None if best is None else best[1]


def post_chat(url, payload, timeout, api_key=None):
    """Send a chat-completions request; the reply's text."""
    headers = {'Content-Type': 'application/json'}
    if api_key:
        headers['Authorization'] = f'Bearer {api_key}'
    request = urllib.request.Request(
        url, data=json.dumps(payload).encode('utf-8'), headers=headers
    )
    with urllib.request.urlopen(request, timeout=timeout) as response:
        body = json.loads(response.read().decode('utf-8'))
    return body['choices'][0]['message']['content']


class VlmClient:
    """Sends one request at a time on a background thread, so the
    explorer's timer (which also watches for obstacles) never waits on the
    network. The explorer polls take_result() on its next loop.
    """

    def __init__(self, url, model, timeout, encode, api_key=None, post=post_chat):
        self.url = url
        self.model = model
        self.timeout = timeout
        self.encode = encode
        self.api_key = api_key
        self.post = post
        self.lock = threading.Lock()
        self.thread = None
        self.result = None

    def busy(self):
        return self.thread is not None and self.thread.is_alive()

    def submit(self, goals, frames, facts=None):
        """Ask about `goals` [(x, y)], with a camera frame (or None) each, and
        optionally one line of what the map says about each."""
        if self.busy():
            return False
        self.thread = threading.Thread(
            target=self._run, args=(list(goals), list(frames), facts), daemon=True
        )
        self.thread.start()
        return True

    def take_result(self):
        """The finished Ranking, once; None while waiting or if taken."""
        with self.lock:
            result, self.result = self.result, None
        return result

    def _run(self, goals, frames, facts=None):
        has_view = [frame is not None for frame in frames]
        try:
            jpegs = [self.encode(frame) for frame in frames if frame is not None]
            seen_facts = facts and [f for f, seen in zip(facts, has_view) if seen]
            text = self.post(
                self.url, build_request(self.model, jpegs, seen_facts), self.timeout,
                self.api_key
            )
            scores, reason = parse_reply(text, len(jpegs))
            if scores is not None:
                scores = merge_scores(has_view, scores)
        except (urllib.error.URLError, OSError, ValueError, KeyError, IndexError) as e:
            scores, reason = None, f'request failed: {e}'
        except Exception as e:  # never let the thread die silently
            scores, reason = None, f'unexpected error: {e!r}'
        with self.lock:
            self.result = Ranking(goals, scores, reason, time.monotonic())
