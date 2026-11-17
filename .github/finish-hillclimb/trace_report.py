"""Why was Oak nose-on to a wall when the obstacle watch cancelled a goal?
(issue #142). Reads the explorer logs of a batch and prints a Markdown table.

    python3 trace_report.py <downloaded artifacts dir>

Each artifact dir is named <arm>-run<NN> and holds
<timestamp>/logs/run_<NN>_explorer.log. The logs need the 'Trace:' lines the
explorer writes when its diagnostic_trace parameter is true.

Every cancel is put in one or more of:
  A  path hugging: Oak is on the plan and facing along it, so the plan itself
     led Oak's nose inside the watch distance.
  B  tracking error: Oak is more than OFF_PLAN_M from the plan.
  C  turn overshoot: Oak's heading is more than OFF_HEADING_RAD off the plan's.
  D  none of these: no trace line or no plan to compare with.
B and C can both be true; A is neither.

Two plans are compared. Nav2 replans from Oak's own pose every second, so the
latest plan (plan_dist, plan_yaw_err) starts at Oak and is close by
construction. The reference plan (ref_dist, ref_yaw_err) is the one Oak had
about 5 s earlier: it shows whether Oak went where it was told.
"""

import glob
import os
import re
import statistics
import sys

OFF_PLAN_M = 0.15
OFF_HEADING_RAD = 0.35
AT_CANCEL_SEC = 1.0     # a trace line this close before a cancel describes it
LOOK_BACK_SEC = 30.0
MEDIAN_SEC = 5.0

LINE = re.compile(r'\[\w+\] \[(\d+\.\d+)\] \[frontier_explorer\]: (.*)')
FIELD = re.compile(r'(\w+)=([^,\s]+)')
GOAL = re.compile(r'Sending (exploration|escape) goal:')
CANCEL = re.compile(r'Obstacle .* ahead for .*abandoning this goal')
NUMBERS = ('plan_dist', 'plan_yaw_err', 'ref_dist', 'ref_yaw_err', 'cmd_v', 'ref_age')


def number(text):
    try:
        return float(text)
    except (TypeError, ValueError):
        return None


def read_cancels(lines):
    """One entry per watch cancel: the trace samples of that goal from the
    LOOK_BACK_SEC before it, each a dict with 'age' (s before the cancel) and
    the NUMBERS (None where the log said none). Also returns goals sent."""
    cancels, samples, goals = [], [], 0
    for line in lines:
        m = LINE.search(line)
        if not m:
            continue
        when, text = float(m.group(1)), m.group(2)
        if GOAL.match(text):
            goals += 1
            samples = []
        elif text.startswith('Trace:'):
            raw = dict(FIELD.findall(text))
            sample = {name: number(raw.get(name)) for name in NUMBERS}
            sample['when'] = when
            samples.append(sample)
        elif CANCEL.match(text):
            cancels.append([dict(s, age=when - s['when']) for s in samples
                            if 0.0 <= when - s['when'] <= LOOK_BACK_SEC])
            samples = []
    return cancels, goals


def letters(sample, prefix):
    """Which of A, B, C, D one trace sample is, against the plan `prefix`."""
    if sample is None:
        return {'D'}
    dist, yaw = sample[f'{prefix}_dist'], sample[f'{prefix}_yaw_err']
    if dist is None or yaw is None:
        return {'D'}
    found = set()
    if dist > OFF_PLAN_M:
        found.add('B')
    if abs(yaw) > OFF_HEADING_RAD:
        found.add('C')
    return found or {'A'}


def at_cancel(samples, prefix):
    recent = [s for s in samples if s['age'] <= AT_CANCEL_SEC]
    return letters(min(recent, key=lambda s: s['age']) if recent else None, prefix)


def any_time(samples, prefix):
    """B or C if any sample of the window was; A only if every one was A."""
    found = set().union(*(letters(s, prefix) for s in samples)) if samples else {'D'}
    if found & {'B', 'C'}:
        return found & {'B', 'C'}
    return {'A'} if found == {'A'} else {'D'}


def window_median(samples, name, absolute=False):
    values = [s[name] for s in samples if s['age'] <= MEDIAN_SEC and s[name] is not None]
    if not values:
        return None
    return statistics.median(abs(v) if absolute else v for v in values)


def report(cancels, goals, logs_read, logs_with_trace):
    n = len(cancels)
    out = ['## Path trace: where Oak was against its plan when the watch cancelled',
           '',
           f'Logs read: {logs_read} ({logs_with_trace} with trace lines). '
           f'Goals sent: {goals}. Watch cancels: {n}'
           + (f' ({100.0 * n / goals:.0f} % of goals).' if goals else '.'),
           '']
    if not n:
        return '\n'.join(out + ['No watch cancels found.'])

    columns = [
        ('at the cancel, plan from 5 s earlier', [at_cancel(c, 'ref') for c in cancels]),
        ('any time in the 30 s before, plan from 5 s earlier',
         [any_time(c, 'ref') for c in cancels]),
        ('at the cancel, latest plan', [at_cancel(c, 'plan') for c in cancels]),
    ]
    rows = [
        ('A path hugging (on the plan, facing along it)', lambda f: f == {'A'}),
        (f'B tracking error (more than {OFF_PLAN_M} m off the plan)', lambda f: 'B' in f),
        (f'C turn overshoot (heading more than {OFF_HEADING_RAD} rad off)',
         lambda f: 'C' in f),
        ('B and C together', lambda f: f >= {'B', 'C'}),
        ('D none of these (no trace line or no plan)', lambda f: f == {'D'}),
    ]
    out.append('| share of cancels | ' + ' | '.join(name for name, _ in columns) + ' |')
    out.append('|---|' + '---:|' * len(columns))
    for label, test in rows:
        cells = []
        for _, found in columns:
            count = sum(test(f) for f in found)
            cells.append(f'{100.0 * count / n:.0f} % ({count})')
        out.append(f'| {label} | ' + ' | '.join(cells) + ' |')
    out += ['',
            f'Median over the {MEDIAN_SEC:.0f} s before a cancel '
            '(median per cancel, then across cancels):',
            '',
            '| measure | median | cancels with data |',
            '|---|---:|---:|']
    for label, name, absolute in (
        ('plan_dist, m (latest plan)', 'plan_dist', False),
        ('plan_yaw_err, rad, absolute (latest plan)', 'plan_yaw_err', True),
        ('ref_dist, m (plan from 5 s earlier)', 'ref_dist', False),
        ('ref_yaw_err, rad, absolute (plan from 5 s earlier)', 'ref_yaw_err', True),
        ('cmd_v, m/s', 'cmd_v', False),
        ('ref_age, s (how old the earlier plan really was)', 'ref_age', False),
    ):
        values = [v for v in (window_median(c, name, absolute) for c in cancels)
                  if v is not None]
        middle = f'{statistics.median(values):.2f}' if values else 'none'
        out.append(f'| {label} | {middle} | {len(values)} |')
    out += ['',
            'B and C can both be true, so the shares can add up to more than 100 %. '
            'Nav2 replans from Oak every second, so the latest-plan column leans to A '
            'by construction; read the 5-s-earlier columns for B and C.']
    return '\n'.join(out)


def find_logs(root):
    """The explorer log of each <arm>-run<NN> artifact (largest: a retried
    startup leaves an empty first log)."""
    logs = []
    for art in sorted(os.listdir(root)):
        if not re.fullmatch(r'.+-run\d+', art):
            continue
        found = glob.glob(os.path.join(root, art, '**', '*_explorer.log'), recursive=True)
        if found:
            logs.append(max(found, key=os.path.getsize))
    return logs


def main():
    cancels, goals, with_trace = [], 0, 0
    logs = find_logs(sys.argv[1])
    for path in logs:
        with open(path, errors='replace') as f:
            lines = f.readlines()
        with_trace += any('Trace:' in line for line in lines)
        found, sent = read_cancels(lines)
        cancels += found
        goals += sent
    print(report(cancels, goals, len(logs), with_trace))


if __name__ == '__main__':
    main()
