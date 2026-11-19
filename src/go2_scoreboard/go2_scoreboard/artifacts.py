"""Atomic scorecard artifact generation."""

import csv
import json
import os
import tempfile

from go2_scoreboard.map_metrics import write_map


CSV_COLUMNS = [
    'run_id', 'source', 'run_label', 'code_version', 'source_hash', 'state',
    'duration_s', 'distance_m', 'known_area_m2', 'completeness_pct',
    'accuracy_error_cm', 'goals_sent', 'goals_reached', 'goals_failed',
    'goals_cancelled', 'recoveries', 'collisions', 'safety_stops',
    'human_touches', 'cpu_peak_pct', 'memory_peak_mb', 'end_reason', 'warnings',
]


def _atomic_text(path, text):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix='.scoreboard-',
                                     dir=os.path.dirname(os.path.abspath(path)))
    try:
        with os.fdopen(fd, 'w', encoding='utf-8', newline='') as stream:
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def _csv_text(rows, columns):
    import io
    output = io.StringIO(newline='')
    writer = csv.DictWriter(output, fieldnames=columns, extrasaction='ignore')
    writer.writeheader()
    for row in rows:
        normalized = dict(row)
        if isinstance(normalized.get('warnings'), list):
            normalized['warnings'] = '; '.join(normalized['warnings'])
        writer.writerow(normalized)
    return output.getvalue()


def report_markdown(summary):
    def value(key, suffix=''):
        return ('n/a' if summary.get(key) is None
                else f"{summary[key]}{suffix}")
    warnings = summary.get('warnings') or []
    lines = [
        f"# Mapping run {summary['run_id']}",
        '',
        f"- State: **{summary['state']}**",
        f"- Source: `{summary['source']}`",
        f"- Configuration: `{summary.get('run_label') or 'default'}`",
        f"- Code: `{summary.get('code_version') or 'unknown'}`",
        f"- Duration: {value('duration_s', ' s')}",
        f"- Distance: {value('distance_m', ' m')}",
        f"- Known map area: {value('known_area_m2', ' m²')}",
        f"- Completeness: {value('completeness_pct', ' %')}",
        f"- Accuracy error: {value('accuracy_error_cm', ' cm')}",
        f"- Goals: {summary['goals_reached']} reached / {summary['goals_sent']} sent, "
        f"{summary['goals_failed']} failed, {summary['goals_cancelled']} cancelled",
        f"- Recoveries: {summary['recoveries']}",
        f"- Collisions: {summary['collisions']}",
        f"- Safety stops: {summary['safety_stops']}",
        f"- Human touches: {summary['human_touches']}",
        f"- Peak container load: {value('cpu_peak_pct', ' % CPU')}, "
        f"{value('memory_peak_mb', ' MB RAM')}",
        f"- End reason: {summary.get('end_reason') or 'n/a'}",
    ]
    if warnings:
        lines.extend(['', '## Warnings', ''] + [f'- {warning}' for warning in warnings])
    return '\n'.join(lines) + '\n'


def write_artifacts(run_dir, accumulator, append_index=True):
    """Write a complete, internally consistent snapshot of one run."""
    os.makedirs(run_dir, exist_ok=True)
    summary = accumulator.summary()
    _atomic_text(os.path.join(run_dir, 'run.json'),
                 json.dumps(summary, indent=2, sort_keys=True) + '\n')
    _atomic_text(os.path.join(run_dir, 'scorecard.csv'),
                 _csv_text([summary], CSV_COLUMNS))
    _atomic_text(os.path.join(run_dir, 'report.md'), report_markdown(summary))
    timeline_columns = [
        'time_s', 'kind', 'event_type', 'goal_sequence', 'detail', 'distance_m',
        'known_area_m2', 'completeness_pct', 'goals_sent', 'goals_reached',
        'goals_failed', 'recoveries', 'collisions', 'human_touches',
    ]
    _atomic_text(os.path.join(run_dir, 'timeline.csv'),
                 _csv_text(accumulator.timeline, timeline_columns))
    process_rows = []
    for row in accumulator.resources:
        if row['processes']:
            for process in row['processes']:
                totals = {
                    key: row[key] for key in ('time_s', 'cpu_pct', 'memory_mb')
                }
                process_rows.append({**totals, **process})
        else:
            process_rows.append({key: row[key] for key in ('time_s', 'cpu_pct', 'memory_mb')})
    resource_columns = ['time_s', 'cpu_pct', 'memory_mb', 'pid', 'name',
                        'process_cpu_pct', 'process_memory_mb']
    _atomic_text(os.path.join(run_dir, 'resources.csv'),
                 _csv_text(process_rows, resource_columns))
    if accumulator.latest_map is not None:
        _atomic_map(accumulator.latest_map, os.path.join(run_dir, 'final_map.yaml'))
    if append_index:
        _update_index(os.path.join(os.path.dirname(run_dir), 'index.csv'), summary)
    return summary


def _atomic_map(grid, yaml_path):
    """Replace both map files only after a complete temporary pair exists."""
    directory = os.path.dirname(os.path.abspath(yaml_path))
    os.makedirs(directory, exist_ok=True)
    temporary_yaml = os.path.join(directory, '.scoreboard-final-map.yaml')
    temporary_pgm = os.path.splitext(temporary_yaml)[0] + '.pgm'
    final_pgm = os.path.splitext(yaml_path)[0] + '.pgm'
    try:
        write_map(grid, temporary_yaml)
        with open(temporary_yaml, encoding='utf-8') as stream:
            metadata = stream.read().replace(
                os.path.basename(temporary_pgm), os.path.basename(final_pgm))
        _atomic_text(temporary_yaml, metadata)
        os.replace(temporary_pgm, final_pgm)
        os.replace(temporary_yaml, yaml_path)
    finally:
        for path in (temporary_yaml, temporary_pgm):
            if os.path.exists(path):
                os.unlink(path)


def _update_index(path, summary):
    rows = []
    if os.path.isfile(path):
        with open(path, newline='', encoding='utf-8') as stream:
            rows = list(csv.DictReader(stream))
    normalized = dict(summary)
    normalized['warnings'] = '; '.join(normalized.get('warnings') or [])
    rows = [row for row in rows if row.get('run_id') != summary['run_id']]
    rows.append(normalized)
    _atomic_text(path, _csv_text(rows, CSV_COLUMNS))
