"""Summarize a finish-hillclimb batch: per arm, runs that finished out of N,
mean map completeness, collisions, and how each unfinished run ended.

    python3 summarize.py <downloaded artifacts dir> <batch.json>

Each artifact dir is named <arm>-run<NN> and holds <timestamp>/results.csv
and <timestamp>/logs/run_<NN>_explorer.log.
"""

import csv
import json
import glob
import os
import re
import sys
from collections import defaultdict

MARKERS = {
    'complete': 'Exploration complete',
    'no_goal': 'No usable goal',
    'stuck': 'Stuck (',
    'no_safe': 'no safe direction',
    'unchanged': 'Frontier unchanged',
    'loop_error': 'Exploration loop error',
    'traceback': 'Traceback',
}


def read_run(art_dir):
    csvs = glob.glob(os.path.join(art_dir, '**', 'results.csv'), recursive=True)
    if not csvs:
        return None
    with open(csvs[0], newline='') as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return None
    row = rows[0]
    logs = glob.glob(os.path.join(os.path.dirname(csvs[0]), 'logs', '*_explorer.log'))
    logs.sort(key=os.path.getsize)  # a retried startup leaves an empty first log
    text = open(logs[-1], errors='replace').read() if logs else ''
    row['counts'] = {k: text.count(v) for k, v in MARKERS.items()}
    row['log_lines'] = text.count('\n')
    return row


def main():
    root = sys.argv[1]
    batch = json.load(open(sys.argv[2]))
    arms = defaultdict(dict)
    missing = [f'{arm}-run{i}' for arm in batch['arms'] for i in range(batch['runs'])
               if not os.path.isdir(os.path.join(root, f'{arm}-run{i}'))]
    for art in sorted(os.listdir(root)):
        m = re.fullmatch(r'(.+)-run(\d+)', art)
        if not m:
            continue
        arm, idx = m.group(1), int(m.group(2))
        row = read_run(os.path.join(root, art))
        if row is None:
            missing.append(art)
            continue
        arms[arm][idx] = row
    out = []
    for arm in batch['arms']:
        runs = arms[arm]
        n = batch['runs']
        done = sum(r['finished'] == 'yes' for r in runs.values())
        comp = [float(r['completeness_pct']) for r in runs.values()] + [0.0] * (n - len(runs))
        coll = sum(int(r['collisions']) for r in runs.values())
        errors = sum(r['counts']['traceback'] + r['counts']['loop_error'] for r in runs.values())
        out.append(f'## {arm}: finished {done}/{n}, mean completeness '
                   f'{sum(comp) / max(n, 1):.1f} %, collisions {coll}, '
                   f'explorer errors {errors}\n')
        out.append('| run | seed | done | complete % | time s | coll | dist m | end reason '
                   '| no-goal | stuck | no-safe | unchanged | log lines |')
        out.append('|---:|---:|---|---:|---:|---:|---:|---|---:|---:|---:|---:|---:|')
        for idx, r in sorted(runs.items()):
            c = r['counts']
            out.append(f"| {idx + 1} | {r['seed']} | {r['finished']} | {r['completeness_pct']} "
                       f"| {r['time_s']} | {r['collisions']} | {r['distance_m']} "
                       f"| {r['end_reason']} | {c['no_goal']} | {c['stuck']} | {c['no_safe']} "
                       f"| {c['unchanged']} | {r['log_lines']} |")
        out.append('')
    if missing:
        out.append(f'Runs with no results (count as not finished): {", ".join(missing)}')
    print('\n'.join(out))


if __name__ == '__main__':
    main()
