"""Print a Markdown receipt of what a cloud Claude run did, read from its transcript.

The transcript is the action's `execution_file`: every message of the run. The
receipt counts tool calls, so it shows what happened, not what the run says it did.
Only names and counts are printed. The transcript itself is never published.
"""

import json
import re
import sys
from collections import Counter


def load(path):
    text = open(path, encoding='utf-8').read().strip()
    try:
        data = json.loads(text)
        return data if isinstance(data, list) else [data]
    except json.JSONDecodeError:
        return [json.loads(line) for line in text.splitlines() if line.strip().startswith('{')]


def tool_calls(messages):
    for message in messages:
        if message.get('type') != 'assistant' or message.get('parent_tool_use_id'):
            continue
        for block in message.get('message', {}).get('content', []) or []:
            if isinstance(block, dict) and block.get('type') == 'tool_use':
                yield block.get('name', ''), block.get('input') or {}


def receipt(messages):
    calls = list(tool_calls(messages))
    plugins = set()
    for message in messages:
        if message.get('type') == 'system' and message.get('subtype') == 'init':
            plugins |= {p.get('name') for p in message.get('plugins') or [] if isinstance(p, dict)}
    result = next((m for m in reversed(messages) if m.get('type') == 'result'), {})

    skills = Counter(i.get('skill', '?') for name, i in calls if name == 'Skill')
    skill_files, playbooks = Counter(), Counter()
    for name, i in calls:
        path = str(i.get('file_path', ''))
        if name != 'Read':
            continue
        found = re.search(r'/skills/([^/]+)/SKILL\.md$', path)
        if found:
            skill_files[found.group(1)] += 1
        found = re.search(r'/playbooks/([^/]+)\.md$', path)
        if found:
            playbooks[('project: ' if '/.agents/' in path else '') + found.group(1)] += 1
    agents = Counter(
        f"{i.get('subagent_type', 'default')} on {i.get('model', 'the session model')}"
        for name, i in calls if name in ('Agent', 'Task'))

    def is_test_run(name, i):
        return name == 'Bash' and 'pytest' in str(i.get('command', ''))

    def is_source_edit(name, i):
        path = str(i.get('file_path', ''))
        return name in ('Edit', 'Write', 'MultiEdit', 'NotebookEdit') and '/src/' in '/' + path \
            and '/test/' not in path and '/tests/' not in path

    first_test = next((n for n, c in enumerate(calls) if is_test_run(*c)), None)
    first_edit = next((n for n, c in enumerate(calls) if is_source_edit(*c)), None)
    if first_edit is None:
        order = 'no source file was edited'
    elif first_test is None:
        order = 'NO: source was edited and no test ever ran'
    elif first_test < first_edit:
        order = 'yes'
    else:
        order = 'NO: source was edited before any test ran'

    def names(counter):
        return ', '.join(f'`{k}`' + (f' x{v}' if v > 1 else '') for k, v in counter.most_common()) or 'none'

    def yes(flag):
        return 'yes' if flag else 'NO'

    called = set(skills) | {f'pstack:{k}' for k in skill_files}
    principles = sorted(k for k in called if 'principle-' in k)
    rows = [
        ('pstack plugin loaded', yes('pstack' in plugins)),
        ('`poteto-mode` skill called', yes('pstack:poteto-mode' in skills or 'poteto-mode' in skill_files)),
        ('Skills called', names(skills)),
        ('Skill files read', names(skill_files)),
        ('Playbooks read', names(playbooks)),
        ('Principles read', ', '.join(f'`{p}`' for p in principles) or 'none'),
        ('Subagents started', f'{sum(agents.values())}' + (f' ({names(agents)})' if agents else '')),
        ('A test ran before the first source edit', order),
        ('Test runs', str(sum(1 for c in calls if is_test_run(*c)))),
        ('Tool calls', str(len(calls))),
        ('Turns', str(result.get('num_turns', '?'))),
        ('Permission denials', str(len(result.get('permission_denials') or []))),
        ('Size at API prices (not a charge)', f"${result.get('total_cost_usd', 0):.2f}"),
    ]
    out = ['### pstack receipt', '',
           'Read from the run transcript by a script, not written by Claude.', '',
           '| Check | Result |', '| --- | --- |']
    out += [f'| {a} | {b} |' for a, b in rows]
    return '\n'.join(out)


if __name__ == '__main__':
    print(receipt(load(sys.argv[1])))
