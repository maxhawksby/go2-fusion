"""Run one trial of a go2_sim batch: the same seed and start pose that run
number --index would get inside `ros2 run go2_sim trials --runs N --seed S`.

Every arm runs this same file, so the harness stays fixed while the code
under test changes.
"""

import os
import sys

import numpy as np

from go2_sim import trials, world_tools


def main():
    index = int(sys.argv[1])
    argv = sys.argv[2:]

    def only_this_run(args, world_path, gt, gt_path, out_dir, header, launch_extra, results):
        run_seed = args.seed * 1000 + index
        pose = world_tools.sample_start_pose(gt, np.random.default_rng(run_seed),
                                             clearance=args.clearance)
        print(f'[one_run] run {index + 1}: seed {run_seed}, start {pose}', flush=True)
        # trials.run_all places the seeded floor objects; this function replaces
        # it, so it has to place the same ones.
        if getattr(args, 'floor_objects', 0) > 0:
            spots = world_tools.place_floor_objects(
                gt, np.random.default_rng(run_seed + 500), pose, count=args.floor_objects)
            print(f'[one_run] floor objects at {spots}', flush=True)
            with open(world_path) as f:
                text = world_tools.add_floor_objects(f.read(), spots)
            world_path = os.path.join(out_dir, f'run_{index + 1:02d}_{os.path.basename(world_path)}')
            with open(world_path, 'w') as f:
                f.write(text)
        # A simulator stack that never comes up (SLAM or Nav2 dies before the
        # auto mapper starts) says nothing about the code under test: retry
        # it once on a fresh ROS domain.
        for attempt in range(2):
            r = trials.run_one(args, index + 20 * attempt, world_path, gt, gt_path, pose,
                               run_seed, out_dir, launch_extra)
            r['run'] = index + 1
            if 'startup' not in r['end_reason']:
                break
            print(f'[one_run] attempt {attempt + 1}: {r["end_reason"]}; retrying', flush=True)
        results.append(r)
        trials.write_results(out_dir, results, header)

    trials.run_all = only_this_run
    trials.main(argv)


if __name__ == '__main__':
    main()
