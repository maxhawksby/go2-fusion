import assert from 'node:assert/strict';
import { promises as fs } from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import test from 'node:test';

import {
  createDashboardServer,
  discoverRuns,
  parseCsv,
  resolveRunDirectory,
} from '../server.mjs';


async function withTemporaryDirectory(callback) {
  const directory = await fs.mkdtemp(path.join(os.tmpdir(), 'oak-scoreboard-ui-'));
  try {
    return await callback(directory);
  } finally {
    await fs.rm(directory, { recursive: true, force: true });
  }
}


test('CSV parser preserves quoted commas and escaped quotes', () => {
  const rows = parseCsv('time_s,detail\n1.0,"goal, accepted"\n2.0,"said ""stop"""\n');
  assert.deepEqual(rows, [
    { time_s: '1.0', detail: 'goal, accepted' },
    { time_s: '2.0', detail: 'said "stop"' },
  ]);
});


test('run paths cannot escape the results root', () => {
  const root = path.resolve('results');
  assert.equal(resolveRunDirectory(root, 'batch/run-1'), path.join(root, 'batch', 'run-1'));
  assert.throws(() => resolveRunDirectory(root, '../secrets'), /escapes/);
  assert.throws(() => resolveRunDirectory(root, ''), /required/);
});


test('run discovery finds nested scorecards and ignores bags', async () => {
  await withTemporaryDirectory(async (root) => {
    const runDirectory = path.join(root, 'trial', 'scoreboard', 'run-1');
    const bagDirectory = path.join(root, 'trial', 'bag', 'not-a-run');
    await fs.mkdir(runDirectory, { recursive: true });
    await fs.mkdir(bagDirectory, { recursive: true });
    await fs.writeFile(path.join(runDirectory, 'run.json'), JSON.stringify({
      run_id: 'run-1', state: 'COMPLETE', completeness_pct: 88.2,
    }));
    await fs.writeFile(path.join(bagDirectory, 'run.json'), JSON.stringify({ run_id: 'ignore-me' }));
    const runs = await discoverRuns(root);
    assert.equal(runs.length, 1);
    assert.equal(runs[0].run_id, 'run-1');
    assert.equal(runs[0].path, 'trial/scoreboard/run-1');
  });
});


test('HTTP API is read-only and returns discovered runs', async () => {
  await withTemporaryDirectory(async (root) => {
    const runDirectory = path.join(root, 'run-1');
    await fs.mkdir(runDirectory);
    await fs.writeFile(path.join(runDirectory, 'run.json'), JSON.stringify({
      run_id: 'run-1', state: 'RECORDING', warnings: [],
    }));
    const server = createDashboardServer({ resultsRoot: root });
    await new Promise((resolve) => server.listen(0, '127.0.0.1', resolve));
    try {
      const address = server.address();
      const response = await fetch(`http://127.0.0.1:${address.port}/api/runs`);
      assert.equal(response.status, 200);
      const payload = await response.json();
      assert.equal(payload.runs[0].run_id, 'run-1');
      const rejected = await fetch(`http://127.0.0.1:${address.port}/api/runs`, { method: 'POST' });
      assert.equal(rejected.status, 405);
    } finally {
      await new Promise((resolve, reject) => server.close((error) => error ? reject(error) : resolve()));
    }
  });
});
