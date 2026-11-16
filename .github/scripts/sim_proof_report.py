"""Write the simulator proof comment: one batch on a pull request next to the same batch on master.

Each side is a folder holding one trial runner results folder (results.csv and
maps/). A side whose folder or results.csv is missing is reported as failed.
Prints Markdown to stdout. Standard library only.
"""

import argparse
import csv
import glob
import os
from collections import Counter

MARKER = '<!-- sim-proof-report -->'
SIDES = ('master', 'pr')
TITLES = {'master': 'Master', 'pr': 'This pull request'}


def load_side(folder):
    found = sorted(glob.glob(os.path.join(folder or '', '**', 'results.csv'), recursive=True))
    if not folder or not found:
        return None
    with open(found[-1], newline='') as f:
        rows = list(csv.DictReader(f))
    for r in rows:
        r['_dir'] = os.path.dirname(found[-1])
    return rows


def num(r, key):
    try:
        return float(r[key])
    except (KeyError, ValueError):
        return float('nan')


def mean(values):
    values = [v for v in values if v == v]
    return sum(values) / len(values) if values else float('nan')


MEASURES = [
    ('Runs finished ("exploration complete")',
     lambda rows: f"{sum(r['finished'] == 'yes' for r in rows)}/{len(rows)}"),
    ('Mean map complete', lambda rows: f"{mean(num(r, 'completeness_pct') for r in rows):.1f} %"),
    ('Lowest map complete', lambda rows: f"{min(num(r, 'completeness_pct') for r in rows):.1f} %"),
    ('Mean time per run', lambda rows: f"{mean(num(r, 'time_s') for r in rows):.0f} s"),
    ('Collisions (all runs)', lambda rows: f"{sum(int(num(r, 'collisions')) for r in rows)}"),
    ('Mean distance walked', lambda rows: f"{mean(num(r, 'distance_m') for r in rows):.1f} m"),
    ('Slowest real-time factor', lambda rows: f"{min(num(r, 'rtf') for r in rows):.2f}"),
]


def end_reasons(rows):
    return Counter(r['end_reason'] or 'unknown' for r in rows)


def summary_table(sides):
    present = [s for s in SIDES if sides[s]]
    lines = ['| Measure | ' + ' | '.join(TITLES[s] for s in SIDES) + ' |',
             '|---|' + '---:|' * len(SIDES)]
    for label, measure in MEASURES:
        cells = [measure(sides[s]) if s in present else 'failed' for s in SIDES]
        lines.append(f'| {label} | ' + ' | '.join(cells) + ' |')
    reasons = sorted(set().union(*(end_reasons(sides[s]) for s in present)))
    for reason in reasons:
        cells = [str(end_reasons(sides[s])[reason]) if s in present else 'failed' for s in SIDES]
        lines.append(f'| Runs that ended: {reason} | ' + ' | '.join(cells) + ' |')
    return '\n'.join(lines)


def seconds(r):
    return f"{num(r, 'time_s'):.0f} s"


def per_run_table(sides):
    by_run = {s: {r['run']: r for r in sides[s] or []} for s in SIDES}
    runs = sorted(set().union(*by_run.values()), key=int)
    lines = ['| Run | Start (x, y) | Map: master | Map: PR | Time: master | Time: PR '
             '| Ended: master | Ended: PR |',
             '|---:|---|---:|---:|---:|---:|---|---|']
    for run in runs:
        m, p = by_run['master'].get(run), by_run['pr'].get(run)
        first = m or p
        start = f"{first['start_x']}, {first['start_y']}"

        def cell(r, text):
            return text(r) if r else '-'
        lines.append(
            f'| {run} | {start} '
            f"| {cell(m, lambda r: r['completeness_pct'] + ' %')} "
            f"| {cell(p, lambda r: r['completeness_pct'] + ' %')} "
            f"| {cell(m, seconds)} | {cell(p, seconds)} "
            f"| {cell(m, lambda r: r['end_reason'])} "
            f"| {cell(p, lambda r: r['end_reason'])} |")
    return '\n'.join(lines)


def same_starts(sides):
    if not all(sides[s] for s in SIDES):
        return True
    starts = {s: {r['run']: (r['start_x'], r['start_y'], r['start_yaw']) for r in sides[s]}
              for s in SIDES}
    shared = starts['master'].keys() & starts['pr'].keys()
    return all(starts['master'][k] == starts['pr'][k] for k in shared)


def map_gallery(rows, side, images):
    cells = []
    for r in rows:
        name = f"run_{int(r['run']):02d}"
        if not os.path.exists(os.path.join(r['_dir'], 'maps', f'{name}_map.pgm')):
            continue
        cells.append((f"Run {r['run']}: {r['completeness_pct']} %",
                      f'<img src="{images}/{side}/{name}.png" width="200" alt="{side} map {name}">'))
    if not cells:
        return '_No maps were saved._'
    per_row = 5
    lines = []
    for i in range(0, len(cells), per_row):
        group = cells[i:i + per_row]
        lines.append('| ' + ' | '.join(title for title, _ in group) + ' |')
        lines.append('|' + ':---:|' * len(group))
        lines.append('| ' + ' | '.join(img for _, img in group) + ' |')
        lines.append('')
    return '\n'.join(lines)


def report(args):
    sides = {'master': load_side(args.master), 'pr': load_side(args.pr)}
    shas = {'master': args.master_sha[:7], 'pr': args.pr_sha[:7]}
    out = [MARKER, '## Simulator proof', '']
    out.append('A GitHub runner ran the auto mapper in the simulator twice: once with this '
               'pull request and once with master, from the same start spots. '
               "Nobody's laptop or robot was used.")
    out.append('')
    out.append(f'**What ran.** World `{args.world}`, {args.runs} runs per side, seed {args.seed} '
               f'(seed {args.seed} gives the same {args.runs} start spots on both sides). '
               f'Each run gets {args.timeout} s, and ends early if Oak moves less than 0.25 m '
               f'in {args.stall_timeout} s. '
               f"This pull request: `{shas['pr']}`. Master, where this branch started: "
               f"`{shas['master']}`. [Logs and files from this run]({args.run_url}).")
    out.append('')
    for s in SIDES:
        if not sides[s]:
            out.append(f'**The {TITLES[s].lower()} side failed** and has no results. '
                       'Its log is in the run linked above.')
            out.append('')
    if not any(sides.values()):
        out.append(f'\n_Updated for `{shas["pr"]}`._')
        return '\n'.join(out)
    if not same_starts(sides):
        out.append('**Warning.** The two sides did not start from the same spots, '
                   'so the per-run rows below do not compare like with like.')
        out.append('')
    slow = [s for s in SIDES if sides[s] and min(num(r, 'rtf') for r in sides[s]) < 0.9]
    if slow:
        out.append('**Warning.** The simulator ran slower than real time (factor below 0.9) on '
                   + ' and '.join(TITLES[s].lower() for s in slow)
                   + '. The auto mapper times its waits by the wall clock, so slow runs '
                   'score worse than they should.')
        out.append('')
    out += ['### Before and after', '', summary_table(sides), '']
    out += ['### Each run', '', per_run_table(sides), '']
    if args.images:
        if sides['pr']:
            out += ['### Maps from this pull request', '',
                    'White is free space Oak saw, black is walls, gray is never seen.', '',
                    map_gallery(sides['pr'], 'pr', args.images), '']
        if sides['master']:
            out += ['<details><summary>Maps from master</summary>', '',
                    map_gallery(sides['master'], 'master', args.images), '', '</details>', '']
    else:
        out += ['The map pictures are in the run\'s files (linked above), under `maps/`.', '']
    n = min(len(sides[s]) for s in SIDES if sides[s])
    out.append(f'**How sure is this?** Each side ran {n} times. In past tests the same start '
               'spot mapped 48 % once and 99 % another time, so a difference of a few points, '
               'or one more finished run, is within normal run-to-run variation. '
               'This comment reports numbers only. It does not say whether the change is better.')
    out.append('')
    out.append(f'_Updated for `{shas["pr"]}`._')
    return '\n'.join(out)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--pr', help='folder holding the pull request side results')
    parser.add_argument('--master', help='folder holding the master side results')
    parser.add_argument('--pr-sha', required=True)
    parser.add_argument('--master-sha', required=True)
    parser.add_argument('--run-url', required=True)
    parser.add_argument('--images', help='URL prefix holding <side>/run_NN.png')
    parser.add_argument('--world', required=True)
    parser.add_argument('--runs', required=True)
    parser.add_argument('--seed', required=True)
    parser.add_argument('--timeout', required=True)
    parser.add_argument('--stall-timeout', required=True)
    print(report(parser.parse_args()))


if __name__ == '__main__':
    main()
