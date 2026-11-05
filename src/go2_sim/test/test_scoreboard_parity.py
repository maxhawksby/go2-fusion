from go2_sim.trials import scoreboard_parity


def test_parity_accepts_documented_tolerances():
    legacy = {'distance_m': 10.0, 'completeness_pct': 75.0, 'collisions': 2}
    score = {'distance_m': 10.8, 'completeness_pct': 74.2, 'collisions': 2}
    checks, failures = scoreboard_parity(legacy, score)
    assert all(checks.values())
    assert failures == []


def test_parity_names_failed_metrics():
    legacy = {'distance_m': 10.0, 'completeness_pct': 75.0, 'collisions': 2}
    score = {'distance_m': 12.0, 'completeness_pct': 72.0, 'collisions': 3}
    _checks, failures = scoreboard_parity(legacy, score)
    assert failures == ['distance', 'completeness', 'collisions']


def test_parity_handles_nullable_scoreboard_fields():
    legacy = {'distance_m': 1.0, 'completeness_pct': 10.0, 'collisions': 0}
    score = {'distance_m': 1.0, 'completeness_pct': None, 'collisions': 0}
    checks, failures = scoreboard_parity(legacy, score)
    assert not any(checks.values())
    assert failures == ['scoreboard metrics unavailable']
