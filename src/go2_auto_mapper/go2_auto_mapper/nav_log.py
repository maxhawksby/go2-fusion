"""A diary of the explorer's navigation: where each goal was, how it ended and
how much new map it really showed, plus the moments it got stuck.

It keeps track of a run for people reading it afterwards, and it is the training
data for fine-tuning a VLM: the camera view toward a goal, labeled by what walking
there actually revealed. Writes goals.jsonl and one JPEG per goal to a folder. No
ROS here, so it tests on any machine. A failed write warns and never stops the
robot's exploring.
"""

import json
import os
import time


class NavLog:

    def __init__(self, folder, warn=print):
        self.folder = folder
        self.warn = warn
        self.goal = None
        self.count = 0
        self.path = os.path.join(folder, 'goals.jsonl')
        try:
            os.makedirs(folder, exist_ok=True)
        except OSError as e:
            self.warn(f'Navigation log off, cannot make {folder}: {e}')
            self.folder = None

    def goal_sent(self, robot, target, known_cells, frontier_size, score, jpeg=None):
        """A new goal went to Nav2. It also finishes the goal before it: the map
        has now had its scan at that goal, so what it revealed can be counted."""
        self.finish(known_cells)
        self.count += 1
        self.goal = {
            'goal': self.count,
            'sent_at': time.time(),
            'robot': [round(v, 3) for v in robot],
            'target': [round(v, 3) for v in target],
            'known_cells_before': known_cells,
            'frontier_size': frontier_size,
            'map_score': round(score, 1) if score is not None else None,
            'result': None,
            'events': [],
        }
        if jpeg is not None:
            self.goal['image'] = self._write_image(jpeg)

    def goal_ended(self, result):
        """How Nav2 finished: 'reached', 'failed', or 'paused'."""
        if self.goal is not None and self.goal['result'] is None:
            self.goal['result'] = result
            self.goal['seconds'] = round(time.time() - self.goal['sent_at'], 1)

    def event(self, what):
        """Something happened during this goal, such as a back-up."""
        if self.goal is not None:
            self.goal['events'].append(what)

    def finish(self, known_cells):
        """Write the open goal out with the new map cells it led to."""
        if self.goal is None:
            return
        goal, self.goal = self.goal, None
        goal['revealed_cells'] = known_cells - goal['known_cells_before']
        self._append(goal)

    def _write_image(self, jpeg):
        if self.folder is None:
            return None
        name = f'goal_{int(time.time())}_{self.count:04d}.jpg'
        try:
            with open(os.path.join(self.folder, name), 'wb') as f:
                f.write(jpeg)
        except OSError as e:
            self.warn(f'Navigation log: could not save a picture: {e}')
            return None
        return name

    def _append(self, record):
        if self.folder is None:
            return
        try:
            with open(self.path, 'a') as f:
                f.write(json.dumps(record) + '\n')
        except OSError as e:
            self.warn(f'Navigation log: could not write: {e}')


def known_cell_count(data):
    """Cells of an occupancy grid that are no longer unknown (-1)."""
    return sum(1 for cell in data if cell != -1)
