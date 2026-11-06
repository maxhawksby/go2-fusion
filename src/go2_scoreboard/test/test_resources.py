from go2_scoreboard.resources import ResourceSampler


def test_missing_proc_and_cgroup_return_safe_zeroes(tmp_path):
    sampler = ResourceSampler(
        proc_root=str(tmp_path / 'missing-proc'),
        cgroup_root=str(tmp_path / 'missing-cgroup'),
    )
    assert sampler.sample(1.0) == (0.0, 0.0, [])
