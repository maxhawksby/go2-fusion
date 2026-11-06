import assert from 'node:assert/strict';
import test from 'node:test';

import { sortRuns } from '../public/run_sort.mjs';


const runs = [
  { run_id: 'run-2', updated_at: '2026-11-06T12:00:00Z', completeness_pct: 92, duration_s: 120 },
  { run_id: 'run-10', updated_at: '2026-11-06T13:00:00Z', completeness_pct: null, duration_s: 30 },
  { run_id: 'run-1', updated_at: '2026-11-06T11:00:00Z', completeness_pct: 48, duration_s: 60 },
];


test('sorts runs without mutating API order', () => {
  const input = [...runs];
  assert.deepEqual(sortRuns(input, 'newest').map((run) => run.run_id), ['run-10', 'run-2', 'run-1']);
  assert.deepEqual(input, runs);
});


test('coverage sorting keeps unavailable percentages last', () => {
  assert.deepEqual(sortRuns(runs, 'coverage-high').map((run) => run.run_id), ['run-2', 'run-1', 'run-10']);
  assert.deepEqual(sortRuns(runs, 'coverage-low').map((run) => run.run_id), ['run-1', 'run-2', 'run-10']);
});


test('sorts by duration and natural run name', () => {
  assert.deepEqual(sortRuns(runs, 'duration-short').map((run) => run.run_id), ['run-10', 'run-1', 'run-2']);
  assert.deepEqual(sortRuns(runs, 'duration-long').map((run) => run.run_id), ['run-2', 'run-1', 'run-10']);
  assert.deepEqual(sortRuns(runs, 'name').map((run) => run.run_id), ['run-1', 'run-2', 'run-10']);
});
