"""python3 -m pytest .github/finish-hillclimb/test_trace_report.py"""

import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import trace_report  # noqa: E402

T0 = 1731790000.0


def info(t, text, level='INFO'):
    return f'[{level}] [{T0 + t:.9f}] [frontier_explorer]: {text}\n'


def goal(t, kind='exploration'):
    return info(t, f'Sending {kind} goal: x=1.00, y=2.00')


def trace(t, plan=(0.02, 0.01), ref=(0.02, 0.01), cmd_v=0.25):
    def pair(values):
        return ('none', 'none') if values is None else (f'{values[0]:.2f}', f'{values[1]:.2f}')
    (pd, py), (rd, ry) = pair(plan), pair(ref)
    return info(t, f'Trace: t={t:.2f}, x=0.00, y=0.00, yaw=0.00, cmd_v={cmd_v:.2f}, '
                   f'cmd_w=0.00, plan_dist={pd}, plan_yaw_err={py}, gap=0.20, '
                   f'plan_age=0.40, ref_dist={rd}, ref_yaw_err={ry}, ref_age=5.20')


def cancel(t):
    return (info(t, 'Obstacle 0.15 m ahead for 0.0 s; abandoning this goal and heading '
                    'somewhere open.', level='WARN')
            + info(t, 'Watch cancel pose: x=0.00, y=0.00, yaw=0.00, forward=0.25'))


def classify(log):
    cancels, _ = trace_report.read_cancels(log.splitlines(keepends=True))
    return [(trace_report.at_cancel(c, 'ref'), trace_report.any_time(c, 'ref'),
             trace_report.at_cancel(c, 'plan')) for c in cancels]


def test_on_the_plan_and_aligned_is_path_hugging():
    log = goal(0) + trace(9.0) + trace(9.5) + trace(10.0) + cancel(10.0)
    assert classify(log) == [({'A'}, {'A'}, {'A'})]


def test_off_the_earlier_plan_is_tracking_error_even_when_the_latest_plan_is_close():
    log = goal(0) + trace(10.0, plan=(0.01, 0.0), ref=(0.30, 0.10)) + cancel(10.0)
    assert classify(log) == [({'B'}, {'B'}, {'A'})]


def test_heading_off_either_way_is_turn_overshoot():
    for yaw_error in (0.5, -0.5):
        log = goal(0) + trace(10.0, ref=(0.05, yaw_error)) + cancel(10.0)
        assert classify(log)[0][0] == {'C'}


def test_off_plan_and_off_heading_is_both():
    log = goal(0) + trace(10.0, ref=(0.40, 0.90)) + cancel(10.0)
    assert classify(log)[0][0] == {'B', 'C'}


def test_exactly_at_the_thresholds_is_still_on_the_plan():
    log = goal(0) + trace(10.0, ref=(0.15, 0.35)) + cancel(10.0)
    assert classify(log)[0][0] == {'A'}


def test_earlier_drift_shows_in_the_30_s_column_only():
    log = goal(0) + trace(3.0, ref=(0.50, 0.0)) + trace(10.0) + cancel(10.0)
    assert classify(log) == [({'A'}, {'B'}, {'A'})]


def test_no_plan_or_no_recent_trace_line_is_none_of_these():
    no_plan = goal(0) + trace(10.0, plan=None, ref=None) + cancel(10.0)
    stale = goal(0) + trace(5.0) + cancel(10.0)
    untraced = goal(0) + cancel(10.0)
    assert classify(no_plan)[0][0] == {'D'}
    assert classify(stale)[0][0] == {'D'}
    assert classify(untraced) == [({'D'}, {'D'}, {'D'})]


def test_trace_lines_of_an_earlier_goal_do_not_count():
    log = (goal(0) + trace(5.0, ref=(0.90, 0.0))
           + goal(6.0, kind='escape') + trace(10.0) + cancel(10.0))
    assert classify(log) == [({'A'}, {'A'}, {'A'})]


def test_trace_lines_older_than_30_s_do_not_count():
    log = goal(0) + trace(1.0, ref=(0.90, 0.0)) + trace(40.0) + cancel(40.0)
    assert classify(log)[0][1] == {'A'}


def test_goals_and_cancels_are_counted():
    log = goal(0) + trace(1.0) + cancel(1.0) + goal(2.0) + goal(30.0) + trace(31.0) + cancel(31.0)
    cancels, goals = trace_report.read_cancels(log.splitlines(keepends=True))
    assert (len(cancels), goals) == (2, 3)


def test_medians_cover_the_5_s_before_the_cancel():
    log = (goal(0)
           + trace(2.0, ref=(0.90, 0.0), cmd_v=0.0)       # 8 s before: left out
           + trace(6.0, ref=(0.10, -0.2), cmd_v=0.10)
           + trace(8.0, ref=(0.20, 0.4), cmd_v=0.20)
           + trace(10.0, ref=(0.30, -0.6), cmd_v=0.30)
           + cancel(10.0))
    (samples,), _ = trace_report.read_cancels(log.splitlines(keepends=True))
    assert trace_report.window_median(samples, 'ref_dist') == 0.20
    assert trace_report.window_median(samples, 'ref_yaw_err', absolute=True) == 0.4
    assert trace_report.window_median(samples, 'cmd_v') == 0.20


def run_report(tmp_path, logs):
    for name, text in logs.items():
        folder = tmp_path / name / '2026-11-17_00-00-00' / 'logs'
        folder.mkdir(parents=True)
        (folder / 'run_01_explorer.log').write_text(text)
    done = subprocess.run(
        [sys.executable, '-I', os.path.join(HERE, 'trace_report.py'), str(tmp_path)],
        capture_output=True, text=True, check=True)
    return done.stdout


def test_report_table_over_a_batch(tmp_path):
    hugging = goal(0) + trace(10.0) + cancel(10.0)
    drifted = goal(0) + trace(10.0, ref=(0.30, 0.0)) + cancel(10.0)
    overshot = goal(0) + trace(10.0, ref=(0.0, 0.6)) + cancel(10.0) + goal(20.0)
    text = run_report(tmp_path, {
        'path_trace-run0': hugging + drifted, 'path_trace-run1': overshot + hugging,
        'summary': ''})
    assert 'Logs read: 2 (2 with trace lines). Goals sent: 5. Watch cancels: 4 (80 % of goals).' in text
    rows = {line.split('|')[1].strip()[0]: [c.strip() for c in line.split('|')[2:5]]
            for line in text.splitlines()
            if line.startswith(('| A', '| B t', '| C', '| D'))}
    assert rows['A'] == ['50 % (2)', '50 % (2)', '100 % (4)']
    assert rows['B'] == ['25 % (1)', '25 % (1)', '0 % (0)']
    assert rows['C'] == ['25 % (1)', '25 % (1)', '0 % (0)']
    assert rows['D'] == ['0 % (0)', '0 % (0)', '0 % (0)']
    assert '| cmd_v, m/s | 0.25 | 4 |' in text


def test_report_on_logs_without_trace_lines_says_so(tmp_path):
    text = run_report(tmp_path, {'path_trace-run0': goal(0) + cancel(10.0)})
    assert 'Logs read: 1 (0 with trace lines)' in text
    assert '| D none of these (no trace line or no plan) | 100 % (1) |' in text


def test_report_with_no_cancels(tmp_path):
    text = run_report(tmp_path, {'path_trace-run0': goal(0) + trace(1.0)})
    assert 'No watch cancels found.' in text
