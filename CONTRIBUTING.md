# Working on this repo

How the team develops: one Docker image that holds only the dependencies, your code mounted into it live, and small branches merged through pull requests.

Using an AI coding agent? It picks up the same rules from [AGENTS.md](AGENTS.md) (Codex, Cursor) or `CLAUDE.md` (Claude Code) automatically.

## One-time setup

```shell
git clone --recursive git@github.com:maxhawksby/go2-fusion.git
cd go2-fusion/docker
docker compose build
```

The first build takes a while (it downloads ROS packages, PyTorch and Open3D). After that the image only rebuilds when a dependency changes.

- Cloned without `--recursive`? Run `git submodule update --init --recursive`.
- On Windows, add `-f docker-compose.windows.yml` to every `docker compose` command. See [WINDOWS_SETUP.md](docs/WINDOWS_SETUP.md).

## Start the robot stack

```shell
cd docker
ROBOT_IP=192.168.50.150 CONN_TYPE=webrtc docker compose up
```

On start the container builds the workspace incrementally, then launches `robot.launch.py`. Run `docker compose build` only after a dependency changes.

If you updated from the older root-level package layout, recreate the container so Compose applies the new `src/` mount. This uses the existing image:

```shell
docker compose down
docker compose up -d
```

Open a shell inside the running container from another terminal:

```shell
docker compose exec unitree_ros bash
```

## The edit loop

Your repo's `src/` folder is mounted into the container at `/ros2_ws/src`. Edit files on your machine as usual; the container sees the change immediately.

| What you changed | What to do |
| --- | --- |
| Python code in a node | Restart the node: Ctrl+C and rerun it, or `docker compose restart` for nodes started by the launch file. No rebuild. |
| C++ code, a new package, or a new executable in `setup.py` | In the container shell: `cbs <package>`, then restart the node |
| A dependency: `package.xml`, `requirements.txt`, `Dockerfile` | `docker compose up --build` |

Why Python needs no rebuild: colcon builds with `--symlink-install`, so `install/` holds shortcuts to your source files rather than copies.

Shortcuts inside the container:

| Command | Does |
| --- | --- |
| `cb` | Build the whole workspace |
| `cbs <pkg> [pkg...]` | Build only the named packages |
| `teleop` | Drive the robot from the keyboard |

If the build ever gets into a strange state, `docker compose down -v` deletes the build output; the next start rebuilds from scratch in about a minute.

## Branches and pull requests

1. Start every piece of work on its own branch. Never commit to `master`. Start the name with your `id` from [docs/team.yml](docs/team.yml), so everyone can see who owns it: `max/fix/nav-obstacle-safety`, `noah/tts-piper-review`.
   ```shell
   git fetch
   git switch -c <your-id>/feat/<short-name> origin/master
   ```
2. Keep it small: one feature or fix per branch, merged within days. If two people need to change the same lines, do one first.
   Write each commit message as one short sentence saying exactly what changed, starting with the part of Oak it touches: `Auto mapper: skip frontiers it can't reach`, not `fix` or `wip`. Anyone reading the branch should know what you're working on without opening the code.
3. Before opening a PR, replay your work on top of the latest `master`:
   ```shell
   git fetch
   git rebase origin/master
   git push -u origin <your-id>/feat/<short-name>    # add --force-with-lease if you pushed before the rebase
   ```
4. Open a pull request. Your own branches are personal sandboxes and need no review; only merging into `master` does: one teammate's approval plus green automatic checks. Then it's merged and the branch deleted.
   - Fill in the template GitHub shows you, a line or two each: **Who** (you, and who should review), **What** it changes, **Why** it's needed, **Before** and **After**, **How to test**, and **Wiki** (the page you updated).
   - Request a teammate as reviewer. The post to the team's Discord channel #feature-revisions (Who, What, Why, Before, After and the link) happens automatically.
5. Everyone else picks it up with `git fetch && git rebase origin/master` on their own branch.

## Keeping the wiki current

To read the wiki as a workspace (page tree, map, who is working on what), run `npm install` once and then `npm run wiki` from the repo folder (needs Node.js 18+ and Python 3.8+, outside Docker). It reloads the page as you edit.

The team docs are a wiki: `README.md` is the center page "Oak", and `docs/wiki/` holds one hub page per area plus one page per program. Each page lists the code it describes, so an automatic check on every pull request notices when you change a program but not its page.

- Starting a task? Open (or reuse) a GitHub issue, add the label `wiki:<page-name>`, and assign yourself, so the wiki graph shows who is working on what.
- Changed a program, or added or removed a package? Update its page in plain English: what it does, what it uses, how it breaks. Write `TODO(owner)` for anything you don't know. A new package gets a new page linked from its hub.
- Check before you open a pull request: `python3 scripts/wiki/check_wiki.py` (or `npm run wiki:check`). It says exactly what to fix.
- If your change really doesn't affect the page (say, a rename), add this line to the pull request description instead: `wiki: no change needed - <reason>`.

Your AI agent does all of this for you through the wiki-keeper steps in [AGENTS.md](AGENTS.md).

## Design rules

- **Extend before adding.** Before creating a new node or topic, check whether an existing one should own it. Robot commands go on `/webrtc_req`, speech on `/tts`. Motion goes on a twist_mux input: each drive source has its own (`/cmd_vel_joy`, `/cmd_vel_keyboard`, `/cmd_vel_mapper` for auto mapper recovery moves, `/cmd_vel` for Nav2, `/cmd_vel_agent`, `/cmd_vel_object_nav`; see `src/go2_robot_sdk/config/twist_mux.yaml`), so a new one gets a new input rather than sharing.
- **One ROS 2 distro for everyone:** Humble.
- **Test without the robot first.** There is one dog and it accepts one connection at a time. Check what your code sends with `ros2 topic echo /cmd_vel_keyboard` (or whichever topic you publish), and save lab time for the final check.
