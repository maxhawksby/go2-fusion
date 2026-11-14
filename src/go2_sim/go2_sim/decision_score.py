"""
Score the auto mapper's VLM decisions against the simulator's true map.

With a navigation log on (nav_log_dir), the explorer writes one record per VLM
ranking to decisions.jsonl: the candidate goals, the map score's pick, the VLM's
pick and the map at that moment. In shadow mode the VLM's pick is never driven to,
so its value has to be worked out. In the simulator the true map is known: from
each candidate, a full lidar sweep on the true map (stopping at real walls) shows
how many cells that were still unknown it would really have revealed. Minus the
explorer's own walking cost, that gives each pick's true value, so every decision
becomes one paired comparison: VLM pick against map pick.

No ROS here, so it tests on any machine.

    score_decisions RUN_DIR [RUN_DIR ...]

where each RUN_DIR holds nav/decisions.jsonl and the trial runner's output folder
(results.csv with the start pose, and the world's .gt.yaml true map). One run per
RUN_DIR: the scorer takes the first start pose and the newest true map it finds, so run
the trial runner with --runs 1 and its own nav_log_dir per run (as --first-index allows).
A score is what a full sweep at the goal would show, not whether Oak could reach it.
"""

import argparse
import csv
import glob
import json
import math
import os
import statistics

import numpy as np

from go2_sim import world_tools

# A full sweep: dense enough that far cells are not missed between rays.
TRUE_RAYS = 360
# The explorer's info_gain_range and info_gain_cells_per_meter defaults.
RANGE_M = 4.0
CELLS_PER_METER = 100.0
# world_tools.read_map marks cells outside the building with this value.
GT_UNKNOWN = 255


def map_to_world(x, y, start_pose):
    """Turn a point in SLAM's map frame into world coordinates."""
    x0, y0, yaw0 = start_pose
    c, s = math.cos(yaw0), math.sin(yaw0)
    return x0 + c * x - s * y, y0 + s * x + c * y


def world_to_map(x, y, start_pose):
    """Turn a world point into SLAM's map frame (where Oak started)."""
    x0, y0, yaw0 = start_pose
    c, s = math.cos(yaw0), math.sin(yaw0)
    return c * (x - x0) + s * (y - y0), -s * (x - x0) + c * (y - y0)


def unknown_in_snapshot(snapshot, x, y):
    """Return True if map-frame point (x, y) was unknown (or off the map) then."""
    data, resolution, (origin_x, origin_y) = snapshot
    row = int(math.floor((y - origin_y) / resolution))
    col = int(math.floor((x - origin_x) / resolution))
    rows, cols = data.shape
    if not (0 <= row < rows and 0 <= col < cols):
        return True
    return data[row, col] < 0


def true_reveal(gt, snapshot, start_pose, x, y, rays=TRUE_RAYS, range_m=RANGE_M):
    """
    Count the cells a sweep from map point (x, y) would newly see on the true map.

    Rays stop at true walls and at the edge of the true map. A cell counts once,
    and only if it was unknown in the snapshot and lies inside the building.
    """
    wx, wy = map_to_world(x, y, start_pose)
    step = gt.resolution
    rows, cols = gt.shape
    seen = set()
    for i in range(rays):
        angle = 2.0 * math.pi * i / rays
        dx, dy = math.cos(angle), math.sin(angle)
        for k in range(1, int(range_m / step) + 1):
            px, py = wx + dx * step * k, wy + dy * step * k
            row, col = gt.world_to_cell(px, py)
            if not (0 <= row < rows and 0 <= col < cols):
                break
            value = gt.data[row, col]
            if value == GT_UNKNOWN:
                break   # outside the building
            if (row, col) not in seen and unknown_in_snapshot(
                    snapshot, *world_to_map(px, py, start_pose)):
                seen.add((row, col))
            if value == world_tools.OCCUPIED:
                break
    return len(seen)


def score_decision(record, snapshot, gt, start_pose, cells_per_meter=CELLS_PER_METER,
                   range_m=RANGE_M):
    """
    Return the true value of every candidate and of both picks for one decision.

    A candidate's value is the cells it would really reveal minus the explorer's
    walking cost, the same trade-off the map score makes with its estimate.
    """
    values = []
    for c in record['candidates']:
        revealed = true_reveal(gt, snapshot, start_pose, c['x'], c['y'], range_m=range_m)
        values.append(revealed - cells_per_meter * c['distance_m'])
    best = max(values)
    return {
        'values': values,
        'map': values[record['map_pick']],
        'vlm': values[record['vlm_pick']],
        'best': best,
        'agree': record['map_pick'] == record['vlm_pick'],
    }


def read_snapshot(path):
    saved = np.load(path)
    return saved['data'], float(saved['resolution']), tuple(saved['origin'])


def read_start_pose(results_csv):
    with open(results_csv) as f:
        row = next(csv.DictReader(f))
    return float(row['start_x']), float(row['start_y']), float(row['start_yaw'])


def score_run(run_dir, cells_per_meter=CELLS_PER_METER, range_m=RANGE_M):
    """Score every decision of one run; return a list of score dicts."""
    nav = os.path.join(run_dir, 'nav')
    results = sorted(glob.glob(os.path.join(run_dir, '*', 'results.csv')))
    truths = sorted(glob.glob(os.path.join(run_dir, '*', '*.gt.yaml')))
    decisions = os.path.join(nav, 'decisions.jsonl')
    if not (results and truths and os.path.exists(decisions)):
        return []
    gt = world_tools.read_map(truths[-1])
    start_pose = read_start_pose(results[-1])
    scores = []
    with open(decisions) as f:
        for line in f:
            record = json.loads(line)
            if not record.get('map'):
                continue
            snapshot = read_snapshot(os.path.join(nav, record['map']))
            score = score_decision(record, snapshot, gt, start_pose, cells_per_meter, range_m)
            score.update(run=os.path.basename(run_dir.rstrip('/')),
                         decision=record['decision'])
            scores.append(score)
    return scores


def summarize(scores):
    """Return a plain-text summary of paired VLM-pick minus map-pick values."""
    if not scores:
        return 'No scored decisions.'
    differ = [s for s in scores if not s['agree']]
    lines = [f'{len(scores)} decisions, VLM picked differently in {len(differ)}.']
    if differ:
        diffs = [s['vlm'] - s['map'] for s in differ]
        wins = sum(d > 0 for d in diffs)
        losses = sum(d < 0 for d in diffs)
        lines.append(
            f'Where they differ, VLM pick minus map pick (true value, cells): '
            f'mean {statistics.mean(diffs):+.1f}, median {statistics.median(diffs):+.1f}; '
            f'VLM better {wins}, worse {losses}, tied {len(diffs) - wins - losses}.')
        if len(diffs) > 1:
            se = statistics.stdev(diffs) / math.sqrt(len(diffs))
            lines.append(f'Rough 95% range of the mean: {statistics.mean(diffs) - 1.96 * se:+.1f}'
                         f' to {statistics.mean(diffs) + 1.96 * se:+.1f}.')
    for name in ('map', 'vlm'):
        hit = sum(s[name] >= s['best'] for s in scores)
        lines.append(f'{name} pick was the truly best candidate in {hit} of {len(scores)}.')
    return '\n'.join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('run_dirs', nargs='+')
    parser.add_argument('--csv', help='also write every scored decision here')
    parser.add_argument('--range-m', type=float, default=RANGE_M,
                        help="sweep range (m); match the explorer's info_gain_range")
    parser.add_argument('--cells-per-meter', type=float, default=CELLS_PER_METER,
                        help="walking cost; match the explorer's info_gain_cells_per_meter")
    args = parser.parse_args(argv)
    scores = [s for d in args.run_dirs
              for s in score_run(d, args.cells_per_meter, args.range_m)]
    if args.csv:
        with open(args.csv, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['run', 'decision', 'agree', 'map', 'vlm', 'best'])
            for s in scores:
                writer.writerow([s['run'], s['decision'], s['agree'],
                                 round(s['map'], 1), round(s['vlm'], 1), round(s['best'], 1)])
    print(summarize(scores))


if __name__ == '__main__':
    main()
