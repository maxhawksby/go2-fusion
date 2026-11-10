"""An interrupted batch must not repeat completed seeds or change later ones."""

import numpy as np
import pytest

from go2_sim.trials import trial_seeds


def test_resuming_after_two_runs_keeps_exact_remaining_random_streams():
    original = list(trial_seeds(28, 5))
    remaining = list(trial_seeds(28, 3, start_index=2))
    assert remaining == [28002, 28003, 28004]
    assert original[:2] + remaining == original
    assert set(original[:2]).isdisjoint(remaining)
    original_samples = [np.random.default_rng(seed).random(3) for seed in original]
    resumed_samples = [np.random.default_rng(seed).random(3) for seed in remaining]
    assert np.array_equal(original_samples[2:], resumed_samples)


def test_default_offset_preserves_existing_batch_schedule():
    assert list(trial_seeds(1, 2)) == [1000, 1001]
    assert list(trial_seeds(28, 1)) == [28000]


def test_negative_offset_is_rejected():
    with pytest.raises(ValueError, match='nonnegative'):
        trial_seeds(28, 3, start_index=-1)
