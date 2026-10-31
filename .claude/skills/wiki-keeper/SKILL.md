---
name: wiki-keeper
description: Keep the team wiki (README.md + docs/wiki/) true to the code. Use before every commit that touches code a wiki page covers, when a folder or program is added or removed, before opening a pull request, when starting a task (to claim it on the wiki graph), when someone asks to update or check the wiki, to answer wiki questions, or to ask a teammate for review.
---

<!-- Made by Hypatia 0.4.0 (#aab99f51). Run "hypatia upgrade" to update it. -->

# Wiki keeper

You are this repo's librarian. The wiki is a graph: README.md is the center page, hub pages hang off it, and each part of the code has one page under a hub, in `docs/wiki/<slug>.md`. The site (`npx hypatia serve`) shows the map, the pages, and who is working on what. Every page starts with front matter:

```
---
title: Search
parent: web-app               # hub pages use "home"
code: [web/src/search/**]     # the repo paths this page documents
verified: 1a2b3c4d5e6f        # written by hypatia verify; never type it by hand
---
```

`verified:` is a stamp of the code under `code:`. It means "a librarian checked this page against today's code". When that code changes, the stamp stops matching and the page is stale; a page with `code:` and no stamp is unverified. `drift:` in `docs/team.yml` decides what happens: `off` ignores it, `warn` lists stale pages, `error` fails the check.

## Privacy (a firm rule, always)

Pages describe how the code works, never real records, user names, emails or keys. That means no rows from a real database, no names or details of the app's users, no email addresses, phone numbers, student or customer IDs, passwords, tokens or API keys, not even as examples. Use made-up placeholders (`user@example.com`, `<api-key>`, "a student"). The team view shows only the repo's GitHub collaborators, never the app's users.

`npx hypatia check` runs a privacy guard: it names the page and line of anything that looks like personal data or a key, and the website build refuses to publish until it is gone. Never silence it by adding a real value to `allow:` in `docs/team.yml`; that list is only for values that are safe to publish (a public support address). A page that must hold sensitive notes gets `private: true` in its front matter: it shows on the local site but never on the website.

## When starting a task

So the wiki graph shows who is working on what:

1. Find an open GitHub issue for the task, or open one: `gh issue list --search "<words>"`, `gh issue create`.
2. Label it `wiki:<slug>` for the page it touches (create the label if missing: `gh label create wiki:<slug>`), and assign yourself: `gh issue edit <n> --add-label wiki:<slug> --add-assignee @me`. If `gh` is not signed in, tell the human the exact commands instead of skipping this.

## Before every commit that touches code a page covers

1. `git fetch origin`, then run `npx hypatia check --base origin/master`. It lists each stale or unverified page and the files under its `code:` that changed.
2. For each page it lists, reread those files (and `git diff origin/master...HEAD -- <paths>`). Rewrite only the sections the change affects; never rewrite a whole page for a small change. Write in plain English a college freshman would follow:
   - what that part does for the project,
   - what it uses (other parts, services, data, hardware),
   - how it breaks (known failure modes and what they look like).
   Not code internals: no function names or line-by-line walkthroughs.
3. Then run `npx hypatia verify docs/wiki/<slug>.md`. Verify only after you actually reread the code. If nothing on the page needed to change, rereading and verifying is still the right move. Never verify a page you did not check.
   `git add` new files first: files git does not track are left out of the stamp.
   If a merge conflicts on a `verified:` line, don't pick a side: reread the page against the merged code and verify again.
4. New part of the code (a new top-level folder, package or service): create `docs/wiki/<slug>.md` with title, parent (its hub) and `code:` paths that cover the whole folder (`code: [src/<folder>/**]`), link it from the hub page, and verify it. With `coverage:` set in `docs/team.yml`, the check fails until every folder there has a page.
5. Never delete or merge pages without asking the human first. A removed part of the code is a question for them, not a cleanup for you.
6. Never invent facts: if you don't know something (a limit, an owner, a failure mode), write `TODO(owner): <question>`. Keep pages short: a few paragraphs or bullets, linking to other pages rather than repeating them.
7. Run `npx hypatia check --base origin/master` again on your own work until it prints "Wiki check passed", then commit the page changes with the code. `npx hypatia check --warn` lists the same problems without failing, for a quick look; the pass that counts is the one without it.
8. If the check says this project's Hypatia setup is from an older version, run `npx hypatia upgrade --dry-run`, show the human the plan and ask before going on (they decide on the drift setting, and whether hand-edited files may be replaced). Only with their okay, run `npx hypatia upgrade --yes` (add `--no-drift` if they do not want it to add `drift: warn` to a config that has no `drift:` line) and commit the result in the same pull request.

Optional, on a laptop: `npx hypatia hooks install` adds a git pre-push hook that prints the check as warnings before each push and never blocks it. Suggest it to the human; don't install it unasked.

Skipping a page is the exception. Prefer verify. Only when a page truly cannot be brought up to date in this pull request, and you can say why honestly, add a line for that one page to the pull request description:

```
wiki: no change needed for docs/wiki/<slug>.md - <reason>
```

Never use it to skip a page that is now wrong. The older line without a page name still works but is going away; do not write it.

## Committing

Write every commit message as one short sentence that says exactly what changed, starting with the part of the project it touches (`Search: rank exact names first`). Never `fix`, `wip` or `updates`. The wiki shows these messages next to each person's branch, so they are how teammates see what you're working on.

## Answering questions

Teammates ask questions on wiki pages from the site ("Ask the team"); each one is a GitHub issue labelled `question` and `wiki:<slug>`.

1. When asked to answer questions, list the open ones for the pages you know: `gh issue list --label question` (narrow with `--label wiki:<slug>`), then `gh issue view <n>`.
2. Read the page and its code, then answer in plain English a newcomer can follow, as a comment on the issue: `gh issue comment <n> --body "..."`. Link the wiki page and the code on GitHub (`https://github.com/maxhawksby/go2-fusion/blob/master/<path>`).
3. If the answer belongs on the page (others will ask it too), update the page in the same pull request as any code change, or in a small wiki-only pull request, and verify it.
4. Never guess. If you don't know, say what you checked and who might know (the page's owner), and leave the question open.

A comment from someone other than the asker marks the question answered on the site.

## Asking for review

Open the pull request (`gh pr create`), using the repo's pull request template if it has one. Either way, say in a line or two each, in plain English:

- **What**: what the change does, in one sentence.
- **Why**: why the project or the team needs it.
- **Before** and **After**: what happened before, what happens now.
- **How to test**: steps a teammate can follow.
- **Wiki**: each page you updated and why, one line each (`docs/wiki/search.md - ranking now prefers exact names`), plus any per-page skip line.

Request a teammate: `gh pr create --reviewer <login>` or `gh pr edit <n> --add-reviewer <login>`. The People list on the site shows who is on the team and their GitHub logins (names, colors and focus lines can be set in `docs/team.yml`). Label nothing extra: the site shows the review request, and when the repo has the wiki-ping workflow the request is also posted to the team's Discord.

## What `npx hypatia check` checks

- every page has a title and a parent that exists, and page names are lowercase words joined by '-';
- every link from a wiki page, and every link into docs/wiki/, works (README.md's links to other repo files only warn);
- every `code:` path or glob matches at least one file in the repo;
- following parents from any page reaches README.md (no orphans, no loops);
- with `drift: warn` or `drift: error`: every page with `code:` has a `verified:` stamp that matches its code (`--base <ref>` adds what changed; `--warn` only lists);
- with `coverage:` in `docs/team.yml`: every top-level folder in those folders is claimed by some page's `code:`;
- warns when a hub page (parent: home) is not reachable by links from README.md, and counts `TODO(` markers.
