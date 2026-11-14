"""
Diary of the explorer's navigation, one line per goal.

A diary of the explorer's navigation: where each goal was, how it ended and how much new map it
really showed, plus the moments it got stuck.

It keeps track of a run for people reading it afterwards, and it is the training data for
fine-tuning a VLM: the camera view toward a goal, labeled by what walking there actually revealed.
Writes goals.jsonl and one JPEG per goal to a folder; with the VLM on, also decisions.jsonl and a
map snapshot per VLM ranking, so each choice can be scored afterwards. No ROS here, so it tests on
any machine. A failed write warns and never stops the robot's exploring.
"""

import json
import os
import time
import uuid


class NavLog:

    def __init__(self, folder, warn=print):
        self.folder = folder
        self.warn = warn
        self.goal = None
        self.count = 0
        self.decisions = 0
        # Part of every snapshot name, so a restart into the same folder can't
        # overwrite the maps earlier decisions point to.
        self.session = f"{time.strftime('%Y%m%d-%H%M%S')}-{uuid.uuid4().hex[:6]}"
        self.path = os.path.join(folder, 'goals.jsonl')
        try:
            os.makedirs(folder, exist_ok=True)
        except OSError as e:
            self.warn(f'Navigation log off, cannot make {folder}: {e}')
            self.folder = None

    def goal_sent(self, robot, target, known_cells, frontier_size, score, jpeg=None):
        """
        Record a new goal sent to Nav2.

        It also finishes the goal before it: the map has now had its scan at that goal, so what it
        revealed can be counted.
        """
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

    def decision(self, robot, candidates, map_pick, vlm_pick, mode, grid):
        """
        Record one VLM ranking: every candidate, the map's pick and the VLM's pick.

        candidates is a list of dicts (x, y, map_score, vlm_score, distance_m). grid is
        (resolution, origin_x, origin_y, width, height, data), the map at this moment, saved
        beside the log so a scorer can tell which cells were still unknown. Written in shadow
        mode too, so each choice can be scored later against what either pick would show.
        """
        if self.folder is None:
            return
        self.decisions += 1
        record = {
            'decision': self.decisions,
            'at': time.time(),
            'goal': self.count,
            'mode': mode,
            'robot': [round(v, 3) for v in robot],
            'candidates': candidates,
            'map_pick': map_pick,
            'vlm_pick': vlm_pick,
            'map': self._write_grid(grid),
        }
        self._append(record, 'decisions.jsonl')

    def _write_grid(self, grid):
        resolution, origin_x, origin_y, width, height, data = grid
        name = f'map_{self.session}_{self.decisions:04d}.npz'
        try:
            import numpy as np
            np.savez_compressed(
                os.path.join(self.folder, name),
                data=np.asarray(data, dtype=np.int8).reshape(height, width),
                resolution=resolution, origin=(origin_x, origin_y))
        except (OSError, ValueError) as e:
            self.warn(f'Navigation log: could not save the map: {e}')
            return None
        return name

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

    def _append(self, record, name='goals.jsonl'):
        if self.folder is None:
            return
        try:
            with open(os.path.join(self.folder, name), 'a') as f:
                f.write(json.dumps(record) + '\n')
        except OSError as e:
            self.warn(f'Navigation log: could not write: {e}')


def known_cell_count(data):
    """Count the cells of an occupancy grid that are known."""
    return sum(1 for cell in data if cell != -1)
