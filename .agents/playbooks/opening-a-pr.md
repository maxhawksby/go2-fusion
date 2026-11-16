---
extends: opening-a-pr
when: Opening a pull request on go2-fusion, or starting the branch that will become one.
---

# Opening a PR in go2-fusion

The rules come from [AGENTS.md](../../AGENTS.md) (Git and Wiki). Where they and the bundled steps disagree, AGENTS.md wins.

- **In** "Work from a git worktree off main." Trunk is `master`. Branch from `origin/master` as `<id>/<kind>/<short-name>`, with `id` from `docs/hypatia.yml`. Ask the human for their id. Never commit to `master`. Claim the task on an issue as the wiki-keeper skill says under "When starting a task".
- **Replace** "Conventional Commits `type(scope): subject`, no trailing period." Write commit and PR titles as one short sentence that starts with the part of Oak it touches, as AGENTS.md (Git) says.
- **Replace** "Use `##` headings, not bold lead-ins, in this order, and drop a section with nothing to say:" Fill every section of `.github/pull_request_template.md` instead of the bundled list below.
- **After** "Run `/deslop` over the diff before commit and `/no-comments` before review." Before you open the PR:
  1. Run `git fetch && git rebase origin/master`.
  2. If the diff touches `src/`, `docker/` or `host_tools/`, run the Docker build check from AGENTS.md (Build and run). Run `docker info` first.
  3. Run `npx hypatia check --base origin/master` and fix pages per the wiki-keeper skill until it passes.
- **After** "Post the URL and keep building." Request no reviewer. Opening the PR posts it to Discord #feature-revisions on its own.
