function numeric(value) {
  if (value === null || value === undefined || value === '') return null;
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : null;
}


function compareNullableNumbers(left, right, direction) {
  const leftValue = numeric(left);
  const rightValue = numeric(right);
  if (leftValue === null && rightValue === null) return 0;
  if (leftValue === null) return 1;
  if (rightValue === null) return -1;
  return (leftValue - rightValue) * direction;
}


function updatedTime(run) {
  const value = new Date(run.updated_at || 0).getTime();
  return Number.isFinite(value) ? value : 0;
}


function runName(run) {
  return String(run.run_label || run.run_id || 'Mapping run');
}


export function sortRuns(runs, order = 'newest') {
  return [...runs].sort((left, right) => {
    let primary = 0;
    if (order === 'oldest') {
      primary = updatedTime(left) - updatedTime(right);
    } else if (order === 'coverage-high') {
      primary = compareNullableNumbers(left.completeness_pct, right.completeness_pct, -1);
    } else if (order === 'coverage-low') {
      primary = compareNullableNumbers(left.completeness_pct, right.completeness_pct, 1);
    } else if (order === 'duration-long') {
      primary = compareNullableNumbers(left.duration_s, right.duration_s, -1);
    } else if (order === 'duration-short') {
      primary = compareNullableNumbers(left.duration_s, right.duration_s, 1);
    } else if (order === 'name') {
      primary = runName(left).localeCompare(runName(right), undefined, { numeric: true });
    } else {
      primary = updatedTime(right) - updatedTime(left);
    }
    return primary
      || updatedTime(right) - updatedTime(left)
      || runName(left).localeCompare(runName(right), undefined, { numeric: true });
  });
}
