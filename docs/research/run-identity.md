# What a run must record to be tied to its code and settings

Research for [issue #79](https://github.com/maxhawksby/go2-fusion/issues/79), part of map [#77](https://github.com/maxhawksby/go2-fusion/issues/77).
Read from master at `f821efb` on 2026-11-11. Nothing was run. No code was changed.
This file lives on the throwaway branch `max/research/run-identity` and is never merged.

All `file:line` references are to master at `f821efb`.

## Short answer

- The scorecard says `unknown` because the recorder never asks Git anything. It reads one environment variable, `GO2_CODE_VERSION`, and nothing in the repo sets it.
- The only real code identity today is the "source fingerprint": a 16-character hash of some file types under `src/`. It can prove two runs used different files. It cannot say which commit, or what the edits were, and it skips the simulator worlds.
- For settings, a run stores only what the person typed as overrides. Every value that came from a default, a yaml file or an environment variable is missing. So are the world name and the time limits.
- Recommended: the host stamps the commit and dirty state into the container through the environment variable the recorder already reads, and the run saves a dump of every node's live parameters. Details in "Options".

## 1. Why `code_version` is "unknown"

The whole chain is four facts.

1. The recorder declares the parameter with the environment as its default: `os.environ.get('GO2_CODE_VERSION', 'unknown')` (`src/go2_scoreboard/go2_scoreboard/recorder.py:56`). It copies that string into the run (`recorder.py:73`). The data class has the same fallback (`accumulator.py:19`).
2. Nothing sets `GO2_CODE_VERSION`. A search of the whole repo finds the name only on `recorder.py:56`. The compose files pass five or six variables and it is not one of them (`docker/docker-compose.yml:6-12`, `docker/docker-compose.windows.yml:6-11`). The Dockerfile sets four `ENV` values, none of them this (`docker/Dockerfile:74-77`). The trial runner does not pass a `code_version` parameter either (`src/go2_sim/go2_sim/trials.py:402-420`). The hand-typed Oak command does not (`docs/wiki/scoreboard-oak-lab-test.md:92`).
3. No scoreboard or trial-runner code calls `git`. `versioning.py` imports only `hashlib` and `os` (`versioning.py:3-4`). So even with Git visible, the value would still be `unknown`.
4. Git is not visible anyway. Only `src/` is mounted (`docker-compose.yml:14`); `.git` sits one folder up, on the host. The image build cannot see it either: `.dockerignore` excludes `.git` (`.dockerignore:1-2`), and the image holds no source by design (`docker/Dockerfile:1-3`).

The wiki's wording, "the container did not see Git metadata" (`docs/wiki/simulation-run-settings.md:48-49`), is true but is the second reason. The first is that the variable is never set.

One useful consequence: the hook already exists. `docker compose run -e GO2_CODE_VERSION=<sha> ...` should flow straight into `run.json`, because the trial runner copies its whole environment to the recorder (`trials.py:346`, `trials.py:421-423`). Not verified by running.

## 2. What the source fingerprint is

`source_fingerprint()` in `src/go2_scoreboard/go2_scoreboard/versioning.py:7-28`, stored as `source_hash` (`recorder.py:74`, `accumulator.py:193`).

What it does:

- Walks `/ros2_ws/src` (the `source_root` parameter, `recorder.py:57`), in sorted order.
- Skips folders named `__pycache__`, `.pytest_cache` and `results` (`versioning.py:13-14`).
- Hashes the relative path and full bytes of every file ending in `.py .cpp .hpp .h .yaml .xml .msg .srv` (`versioning.py:16-24`).
- Returns the first 16 hex characters of the SHA-256 (`versioning.py:28`), or an empty string if the folder is missing or holds no matching file.

What it covers well: all Python (including launch files), C++, messages, `package.xml`, and the Nav2, SLAM, twist_mux and simulator yaml files. It includes uncommitted edits and untracked files, because it reads the disk, not Git.

What it does not cover:

| Gap | Evidence |
| --- | --- |
| Simulator worlds (`.sdf`) and the robot model (`model.sdf.template`) | Not in the suffix list (`versioning.py:16`). Tracked files: `src/go2_sim/worlds/*.sdf`, `src/go2_sim/models/oak/model.sdf.template`. A changed world changes the score and leaves the hash the same. |
| `.urdf`, `CMakeLists.txt`, `setup.py` is covered but `setup.cfg` is not, `.rviz`, `.action`, model weights (`yolov8n.pt`) | Same suffix list. |
| Anything outside `src/`: `docker/`, `requirements.txt`, the image | Only `src/` is walked. |
| What was actually built | It hashes source, not `install/`. C++ binaries live in a Docker volume (`docker-compose.yml:15-16`) and can be stale; the wiki warns of this (`docs/wiki/scoreboard.md:102`). |
| Which commit, and what the edits were | It is a one-way hash. It answers "same or different", nothing more. The wiki already hit this: the saved control fingerprint `5fbbbe92eba44bd1` matches no known commit (`simulation-run-settings.md:47-51`). |
| Cross-machine comparison | It hashes raw bytes. `.gitattributes` forces LF only for `*.sh`, so a Windows checkout with CRLF conversion would hash differently from a Linux checkout of the same commit. Inferred from the code, not tested. |
| Timing | It is computed once when the recorder starts (`recorder.py:74`), after the simulator is already up (`trials.py:368`, then `trials.py:421`). |

It also does not appear in `report.md`, which prints only `code_version` (`artifacts.py:59`). It is in `run.json` and the CSV rows (`artifacts.py:12`).

## 3. What a run records today

### In `run.json` (the scoreboard's record)

Top-level identity fields (`accumulator.py:186-195`): `schema_version`, `run_id`, `source`, `run_label`, `code_version`, `source_hash`, `configuration`.

`configuration` always holds (`recorder.py:85-92`): `source`, `run_label`, `start_pose`, `ground_truth_map`, `record_bag`, `max_odom_step_m`.

`configuration.parameters` is whatever JSON the caller passed in `configuration_json` (`recorder.py:93-101`). The trial runner passes exactly five keys (`trials.py:412-419`):

- `algorithm`: the fixed string `'frontier'`
- `variant`: the `--variant` label
- `seed`: this run's seed
- `launch_args`: the raw words typed after `--`
- `explorer_params`: the raw `--explorer-param name:=value` words

A hand-started Oak run passes no `configuration_json`, so `parameters` is `{}` (`recorder.py:58`, `scoreboard-oak-lab-test.md:92`).

`scorecard.csv` and `index.csv` carry `code_version` and `source_hash` but not `configuration` (`artifacts.py:11-17`; extra keys are dropped at `artifacts.py:38`).

### In the trial runner's own files

`results.md` gets a one-line header: world file name, run count, base seed, timeout, variant, explorer overrides, launch args, ground truth file (`trials.py:631-636`). `results.csv` has the per-run seed and start pose (`trials.py:357-363`). Launch, explorer and scoreboard console logs are saved per run (`trials.py:365-367`).

### In one host script only

`scripts/run_ite_hallways.ps1` writes a `session.json` with mode, variant, seed, image tag, commit, branch, `git status --porcelain` and the command (`run_ite_hallways.ps1:47-53`). It copies the world file (`:55`) and saves `git diff HEAD --binary` as `source.patch` (`:56`). This is the only place a commit is recorded, and it is why the November 10 hallway run has one (`simulation-run-settings.md:90-91`). It is Windows-only, covers one world, sits beside the run rather than in `run.json`, and its patch leaves out untracked files.

## 4. What a run misses

| Missing | Why it matters | Evidence |
| --- | --- | --- |
| Commit and branch | The row cannot be tied to code. | Section 1. |
| Dirty state and the actual edits | Logged runs came from uncommitted edits (`docs/wiki/experiments.md:41-42`). The fingerprint shows "different", not "what". | Section 2. |
| Explorer parameters left at default | About 30 parameters are declared (`src/go2_auto_mapper/go2_auto_mapper/explorer.py:86-163`). Only typed overrides are stored. If a default changes in code, old and new rows look the same. | `trials.py:418` |
| Explorer values taken from the environment | `vlm_mode`, `vlm_url`, `vlm_model` and `nav_log_dir` default to `VLM_MODE`, `VLM_URL`, `VLM_MODEL`, `NAV_LOG_DIR` (`explorer.py:118-124`, `explorer.py:133`). The runner hands its whole environment to the explorer (`trials.py:346`, `trials.py:444-446`). None of it is recorded. | |
| The explorer's real VLM mode in trials | The runner launches the sim with `explore:=false` (`trials.py:355`) and starts the explorer itself (`trials.py:444`). The launch file only hands `vlm_mode` to an explorer it starts itself (`src/go2_sim/launch/sim.launch.py:168-175`). So a recorded `launch_args: ["vlm_mode:=off"]` only sets the camera rate (`sim.launch.py:97-98`). The explorer's mode comes from `VLM_MODE` or its default. The record can say one thing while the node does another. | |
| World | Not in `run.json` at all. Only in the `results.md` header, by file name, with no hash. | `trials.py:413-419`, `trials.py:631` |
| Time limits and runner dials | `--timeout` is in the header only. `--stall-timeout`, `--startup-timeout` and `--clearance` are recorded nowhere, yet the stall rule ends runs (`trials.py:468-470`). | `trials.py:597-606` |
| Launch arguments left at default | Lidar rate, range, samples, noise and the rest have defaults in code (`sim.launch.py:26-52`). Only typed words are stored. The wiki had to re-read old snapshots to recover them and says so: "declared settings, not a recovered live parameter dump" (`simulation-run-settings.md:79-84`). | |
| Which launch file | Fixed in code as `go2_sim sim.launch.py` (`trials.py:353`), which includes `robot.launch.py` with seven fixed arguments (`sim.launch.py:150-155`). Not written to the run. For Oak runs the launch is whatever the person started. | |
| Nav2 and SLAM yaml contents | Loaded from fixed paths (`src/go2_robot_sdk/launch/robot.launch.py:71-72`). Their text is inside the fingerprint but not readable from the run. | |
| Simulator driver dials | `sim.yaml` plus overrides (`sim.launch.py:95`, `sim.launch.py:125-131`, `src/go2_sim/config/sim.yaml:4-30`). Same as above. | |
| Image and build identity | Nothing in `run.json`. The one script records a mutable tag, `go2scoreboard-unitree_ros:latest` (`run_ite_hallways.ps1:10`, `:48`). No image id, no pip or apt versions, no record of whether the build volume was fresh. | |
| Identity on replay | A replay row starts a new accumulator with no code version or hash, so it shows `unknown` and an empty fingerprint, and does not carry over the original run's (`src/go2_scoreboard/go2_scoreboard/replay.py:62-67`). | |
| Machine | No host name, OS or person. | `accumulator.py:186-213` |

## 5. Options

The job splits in two: **code identity** (commit, dirty state) and **settings** (every value that was live). Options A to C are for code. Option D is for settings. They combine.

### A. The host passes the commit in through the environment

Add `GO2_CODE_VERSION=${GO2_CODE_VERSION:-unknown}` to the `environment:` list of the compose files. A small host wrapper (one bash, one PowerShell, or one Python) sets it from `git rev-parse HEAD` plus a `-dirty` mark from `git status --porcelain`, and saves `git diff HEAD` and the list of untracked files next to the results, as `run_ite_hallways.ps1` already does.

- For: smallest change. The recorder already reads the variable (`recorder.py:56`). No Git needed in the container. Works with worktrees. The host is the only place that truly knows the Git state.
- Against: it trusts the person to use the wrapper; a bare `docker compose` still gives `unknown`. The value is fixed when the container is created, but `src/` is a live mount, so a long-lived container can carry a stale commit. Oak runs use exactly that pattern: `up -d`, then `exec` (`scoreboard-oak-lab-test.md:61`, `:92`). Two host shells to keep in step.
- Fix for the stale case: the wrapper also passes the fingerprint it computed on the host (`versioning.py` is plain Python with no ROS imports, and host and container see the same bytes through the mount). The recorder compares it with its own fingerprint and adds a warning when they differ. The recorder should also add a warning whenever the version is `unknown`, so an unstamped row is loud, not silent.

### B. Mount `.git` read-only and ask Git inside the container

Add `../.git:/ros2_ws/.git:ro` and have the recorder run `git rev-parse` and `git status`.

- For: no wrapper to remember. Always fresh, even in a long-lived container.
- Against, each one a real trap:
  - Worktrees break it. In a worktree `.git` is a small file pointing at an absolute host path, which does not exist in the container. The team runs comparisons from worktrees (`simulation-run-settings.md:73-75`, `scoreboard-oak-lab-test.md:37`).
  - Git would see `/ros2_ws` as the checkout, where `docs/`, `docker/` and the rest are missing, so everything outside `src/` shows as deleted. Every status call must be limited to `src`.
  - The container runs as root on files owned by the host user, which Git refuses by default ("dubious ownership") until told otherwise.
  - On Windows checkouts, line-ending conversion set in the host's Git config is invisible to the container's Git, so clean files may show as modified.
  - A read-only mount stops Git refreshing its index, so status may be slow or wrong.
  - Git must be in the image. Not checked.
- It is the most "automatic" option and the most fragile across the team's mixed machines.

### C. Write a build stamp when colcon builds

`entrypoint.sh` already builds on every container start (`docker/entrypoint.sh:25`). It could write a stamp file (time, fingerprint at build) into `install/`.

- For: answers a question nothing else does: "do the built C++ binaries match the source?" That is the stale-volume risk in `scoreboard.md:102`.
- Against: the entrypoint runs inside the container, so it cannot see Git either. It needs A or B to learn the commit. Python edits take effect with no rebuild (`entrypoint.sh:4-5`), so a build stamp goes stale for most of the code. Rebuilds through `cbs` skip the entrypoint (`docker/dev_aliases.sh:6-7`).
- Useful as an add-on check, not as the commit stamp.

### D. Dump the live parameters at run start

Once the explorer has announced `exploration_started`, save every node's parameters into the run folder (for example `params/<node>.yaml`), using `ros2 param dump` or the same parameter services from Python. Also have the trial runner write its own full arguments, the fully resolved launch arguments, the world file's name and hash, and the few environment variables that change behaviour (`VLM_MODE`, `VLM_URL`, `VLM_MODEL`, `NAV_LOG_DIR`, `ROBOT_IP`, `CONN_TYPE`). Never dump secrets (`ROBOT_AES_KEY`, `VLM_API_KEY`).

- For: it records what the nodes actually used, however the value got there: default, yaml, launch argument or environment. That closes the default, environment, Nav2 and SLAM gaps in one move, and it works the same in the simulator and on Oak. "Non-default" then becomes a diff against one saved baseline dump, with no need to teach the code what each default is.
- Against: more files per run. It must run after the explorer is up, and the recorder starts before the explorer (`trials.py:399-400`), so the timing has to hang on the start event. Nav2 has many nodes; a slow or hung node must not block the run. Some settings are not parameters and need the runner to record them: the world, lidar rate, samples and noise are baked into the robot model text (`sim.launch.py:99-106`), and the timeouts live in the runner.

### Recommendation

**A plus D, with C's fingerprint check folded into A.**

- A is the smallest change that gives a true commit and dirty state, and it is the only one that works on every machine and in worktrees. The stale-container hole is closed by comparing fingerprints and warning.
- D is the only option that captures settings as they really were. Recording typed overrides, as today, can be wrong even when it is complete (the `vlm_mode` case in section 4).
- Also widen the fingerprint to cover `.sdf` and the model template, or hash the world on its own. This is one line and removes the worst blind spot.
- Skip B. It fails in worktrees, which the team uses for exactly these comparison runs.

Smallest first step, if only one thing is built: the compose line and a wrapper from A, plus the "unknown" warning. That alone ends unstamped rows.

Open choices this leaves for the spec, not decided here:

- Whether a dirty run is allowed to land in the shared scorecard, or is marked and kept apart.
- Whether the patch for a dirty run is stored with the run, and where.
- Whether one wrapper replaces `run_ite_hallways.ps1`.

## 6. Not verified

Nothing was run. No Docker, no ROS, no robot. Every claim above comes from reading files at `f821efb`.

- That `-e GO2_CODE_VERSION=...` really reaches `run.json`. Read from the code path only.
- Whether `git` is installed in the image. The Dockerfile does not install it itself; the base image may.
- The exact form and speed of `ros2 param dump` on Humble, and how it behaves with Nav2's lifecycle nodes or a hung node.
- The Option B failure modes (worktree pointer, ownership refusal, line endings, read-only index). These are known Git behaviours, reasoned from the mount layout, not reproduced here.
- That a Windows checkout gives a different fingerprint than Linux for the same commit. Inferred from byte hashing and `.gitattributes`.
- That worlds and config in `install/` are live links to `src/` rather than copies. If they are copies, a stale build volume could run an old world while the fingerprint shows the new source.
- Contents of real saved `run.json` files. The results folders are git-ignored (`src/go2_scoreboard/.gitignore:1`, `src/go2_sim/.gitignore:1`) and none were on this machine's checkout.
- Whether teammates have private wrappers that already set the variable. Only the repo was searched.
- Launch files other than `sim.launch.py` and the parts of `robot.launch.py` cited.
