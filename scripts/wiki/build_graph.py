#!/usr/bin/env python3
"""Build site/graph.json (version 2) from the wiki pages, git and GitHub.

Reads README.md (the center node "oak"), docs/wiki/*.md and docs/team.yml,
and writes site/graph.json for the workspace site in site/.

    npm run wiki      # builds, serves http://localhost:4410/ and live-reloads

or by hand: python3 scripts/wiki/build_graph.py, then serve site/.

What goes in (the full shape is in scripts/wiki/README.md):
- pages, their parent and link edges, backlinks, a one-sentence summary and a
  fixed 2D position for each page (same input, same positions);
- the team from docs/team.yml, plus the pages each person has worked on
  (their "past" list and the pages whose code or file they committed to);
- from git: each page's last change, the last 40 commits that touched a
  page or its code, and the active branches (remote branches other than
  master with a commit in the last 14 days, matched to pages by the files
  they change);
- from GitHub, only with GITHUB_TOKEN and GITHUB_REPOSITORY set (as in
  GitHub Actions): open work (issues labelled "wiki:<slug>" and open pull
  requests, matched to pages by the files they change) and questions (issues
  labelled "question" + "wiki:<slug>"). Without a token both are empty.

Standard library only.
"""

import json
import math
import os
import re
import subprocess
import sys
import urllib.error
import urllib.request
from datetime import datetime, timedelta, timezone
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from check_wiki import glob_to_regex  # noqa: E402  (same glob rules as the checker)

ROOT = Path(__file__).resolve().parents[2]
WIKI = ROOT / "docs" / "wiki"
TEAM = ROOT / "docs" / "team.yml"
OUT = ROOT / "site" / "graph.json"
CENTER = "oak"
RECENT_LIMIT = 40
BRANCH_DAYS = 14
BASE_REF = "refs/remotes/origin/master"
DEFAULT_REPO = "maxhawksby/go2-fusion"

# [text](target) but not images ![alt](src)
LINK_RE = re.compile(r"(?<!!)\[[^\]]*\]\(\s*<?([^)\s>]+)>?(?:\s+\"[^\"]*\")?\s*\)")
FENCE_RE = re.compile(r"```.*?```", re.S)


def fail(msg):
    print(f"build_graph: {msg}", file=sys.stderr)
    sys.exit(1)


def warn(msg):
    print(f"build_graph: {msg}", file=sys.stderr)


# ---------------------------------------------------------------- pages

def parse_front_matter(text, path):
    """Return (fields, body). Handles 'key: value' and 'key: [a, b]' lines."""
    if not text.startswith("---"):
        return None, text
    end = text.find("\n---", 3)
    if end == -1:
        fail(f"{path} starts a front matter block with '---' but never closes it.")
    block = text[3:end].strip("\n")
    body = text[end + 4:].lstrip("\n")
    fields = {}
    for line in block.splitlines():
        line = line.split(" #", 1)[0].rstrip()  # drop trailing comments
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        if ":" not in line:
            fail(f"{path}: cannot read front matter line '{line}'.")
        key, value = line.split(":", 1)
        value = value.strip()
        if value.startswith("[") and value.endswith("]"):
            value = [v.strip().strip("'\"") for v in value[1:-1].split(",") if v.strip()]
        else:
            value = value.strip("'\"")
        fields[key.strip()] = value
    return fields, body


def rel(path):
    return path.relative_to(ROOT).as_posix()


def load_pages():
    pages = {}
    readme = ROOT / "README.md"
    pages[CENTER] = {
        "id": CENTER,
        "title": "Oak",
        "parent": None,
        "code": [],
        "file": readme,
        "markdown": readme.read_text(encoding="utf-8"),
    }
    for path in sorted(WIKI.glob("*.md")):
        text = path.read_text(encoding="utf-8")
        fields, body = parse_front_matter(text, rel(path))
        if fields is None:
            continue  # not a wiki page (no front matter)
        slug = path.stem
        if not re.fullmatch(r"[a-z0-9]+(-[a-z0-9]+)*", slug):
            fail(f"{rel(path)}: page names must be lowercase words joined by '-'.")
        if "title" not in fields or "parent" not in fields:
            fail(f"{rel(path)}: front matter needs both 'title' and 'parent'.")
        code = fields.get("code", [])
        if isinstance(code, str):
            code = [code] if code else []
        pages[slug] = {
            "id": slug,
            "title": fields["title"],
            "parent": fields["parent"],
            "code": code,
            "file": path,
            "markdown": body,
        }
    return pages


def page_for(path, pages):
    """Map a resolved file path to a page id, or None if it is not a page."""
    for pid, page in pages.items():
        if page["file"].resolve() == path:
            return pid
    return None


def collect_edges(pages):
    """Return (edges, backlinks). backlinks[slug] = sorted pages that link to it."""
    edges = []
    seen = set()
    backlinks = {pid: set() for pid in pages}

    def add(source, target, kind):
        key = (source, target)
        if source == target or key in seen:
            return
        seen.add(key)
        edges.append({"source": source, "target": target, "kind": kind})

    for pid, page in pages.items():
        parent = page["parent"]
        if parent is not None:
            if parent not in pages:
                fail(f"{rel(page['file'])} says its parent is '{parent}', but there is no page called '{parent}'.")
            add(parent, pid, "parent")

    for pid, page in pages.items():
        text = FENCE_RE.sub("", page["markdown"])
        for target in LINK_RE.findall(text):
            if re.match(r"^[a-z][a-z0-9+.-]*:", target, re.I) or target.startswith("#"):
                continue  # web link or same-page anchor
            target = target.split("#", 1)[0]
            if not target.endswith(".md"):
                continue
            resolved = (page["file"].parent / target).resolve()
            tid = page_for(resolved, pages)
            if tid is None:
                in_wiki = resolved.parent == WIKI.resolve() or resolved == (ROOT / "README.md").resolve()
                if in_wiki or not resolved.exists():
                    fail(f"{rel(page['file'])} links to '{target}', but that page does not exist.")
                continue  # a normal repo doc, not a wiki page
            if tid != pid:
                backlinks[tid].add(pid)
            if (pid, tid) not in seen and (tid, pid) not in seen:
                add(pid, tid, "link")
    return edges, {k: sorted(v) for k, v in backlinks.items()}


def hub_and_depth(pid, pages):
    chain = [pid]
    while pages[chain[-1]]["parent"] is not None:
        nxt = pages[chain[-1]]["parent"]
        if nxt in chain:
            fail(f"the parents of '{pid}' loop back on themselves: {' > '.join(chain + [nxt])}.")
        chain.append(nxt)
    if chain[-1] != CENTER:
        fail(f"'{pid}' does not hang from Oak (its top parent is '{chain[-1]}').")
    depth = len(chain) - 1
    hub = chain[-2] if depth >= 1 else None
    return hub, depth


def summarize(markdown, limit=240):
    """First sentence of the page's prose, as plain text."""
    text = FENCE_RE.sub("", markdown)
    text = re.sub(r"<!--.*?-->", "", text, flags=re.S)
    for para in re.split(r"\n\s*\n", text):
        para = para.strip()
        if not para or para[0] in "#|>-*!<" or re.match(r"^\d+\.\s", para) or para.startswith("Part of:"):
            continue
        para = re.sub(r"!\[[^\]]*\]\([^)]*\)", "", para)
        para = re.sub(r"\[([^\]]*)\]\([^)]*\)", r"\1", para)
        para = re.sub(r"`([^`]*)`", r"\1", para)
        para = re.sub(r"(\*\*|__)(.+?)\1", r"\2", para)
        para = re.sub(r"(?<![\w*])\*([^*]+)\*(?![\w*])", r"\1", para)
        para = " ".join(para.split())
        if not para:
            continue
        # First sentence; a very short one ("Welcome!") takes the next one too.
        parts = re.findall(r".+?[.!?](?=\s|$)|.+$", para)
        sentence = parts[0].strip()
        if len(sentence) < 25 and len(parts) > 1:
            sentence += " " + parts[1].strip()
        if len(sentence) > limit:
            sentence = sentence[:limit - 3].rstrip() + "..."
        return sentence
    return ""


# ---------------------------------------------------------------- layout

def layout(nodes, ring=320.0, step=95.0, min_dist=58.0, iterations=120):
    """Fixed 2D positions: Oak at 0,0, hubs evenly on a ring, each hub's pages
    in a fan pointing away from Oak, then a fixed number of push-apart passes.

    nodes: list of {"id","parent","hub","depth"}. Returns {id: (x, y)}.
    Deterministic: inputs are sorted and every step is plain arithmetic.
    """
    by_id = {n["id"]: n for n in nodes}
    children = {}
    for n in sorted(nodes, key=lambda n: n["id"]):
        if n["parent"] is not None:
            children.setdefault(n["parent"], []).append(n["id"])
    hubs = sorted(n["id"] for n in nodes if n["depth"] == 1)
    pos = {CENTER: (0.0, 0.0)} if CENTER in by_id else {}
    fixed = set(pos) | set(hubs)
    for i, hub in enumerate(hubs):
        a = -math.pi / 2 + 2 * math.pi * i / max(len(hubs), 1)
        pos[hub] = (ring * math.cos(a), ring * math.sin(a))

    def place(parent, angle, spread):
        kids = children.get(parent, [])
        if not kids:
            return
        px, py = pos[parent]
        # Rows of up to 5 per arc, further rows further out.
        per_row = 5
        for k, kid in enumerate(kids):
            row, col = divmod(k, per_row)
            in_row = min(per_row, len(kids) - row * per_row)
            t = 0.0 if in_row == 1 else (col / (in_row - 1) - 0.5)
            a = angle + t * spread
            r = step * (1 + row * 0.8)
            pos[kid] = (px + r * math.cos(a), py + r * math.sin(a))
            place(kid, a, spread * 0.6)

    for hub in hubs:
        hx, hy = pos[hub]
        place(hub, math.atan2(hy, hx), math.radians(150))
    # Anything not reached (should not happen) sits on Oak.
    for n in nodes:
        pos.setdefault(n["id"], (0.0, 0.0))

    ids = sorted(pos)
    for _ in range(iterations):
        moved = False
        for i, a in enumerate(ids):
            for b in ids[i + 1:]:
                ax, ay = pos[a]
                bx, by = pos[b]
                dx, dy = bx - ax, by - ay
                d = math.hypot(dx, dy)
                if d >= min_dist:
                    continue
                if d < 1e-6:  # exactly on top: push along a fixed direction
                    dx, dy, d = 1.0, 0.0, 1.0
                push = (min_dist - d) / 2
                ux, uy = dx / d * push, dy / d * push
                if a in fixed and b in fixed:
                    continue
                if a in fixed:
                    pos[b] = (bx + 2 * ux, by + 2 * uy)
                elif b in fixed:
                    pos[a] = (ax - 2 * ux, ay - 2 * uy)
                else:
                    pos[a] = (ax - ux, ay - uy)
                    pos[b] = (bx + ux, by + uy)
                moved = True
        if not moved:
            break
    return {k: (round(x, 1), round(y, 1)) for k, (x, y) in pos.items()}


# ---------------------------------------------------------------- team

def parse_team(text):
    """Parse docs/team.yml: a 'people:' list of '- key: value' blocks."""
    people, cur = [], None
    for raw in text.splitlines():
        line = raw.split(" #", 1)[0].rstrip() if not raw.lstrip().startswith("#") else ""
        if not line.strip() or line.strip() == "people:":
            continue
        stripped = line.strip()
        if stripped.startswith("- "):
            cur = {}
            people.append(cur)
            stripped = stripped[2:].strip()
        if cur is None or ":" not in stripped:
            continue
        key, value = stripped.split(":", 1)
        value = value.strip()
        if value.startswith("[") and value.endswith("]"):
            cur[key.strip()] = [v.strip().strip("'\"") for v in value[1:-1].split(",") if v.strip()]
        else:
            cur[key.strip()] = value.strip("'\"")
    fields = ("id", "name", "github", "color", "discord_id", "focus")
    out = []
    for p in people:
        if not p.get("id"):
            continue
        rec = {f: p.get(f, "") if isinstance(p.get(f, ""), str) else "" for f in fields}
        past = p.get("past", [])
        rec["past"] = past if isinstance(past, list) else ([past] if past else [])
        out.append(rec)
    return out


def load_team():
    if not TEAM.exists():
        warn(f"no {rel(TEAM)}; building without people.")
        return []
    return parse_team(TEAM.read_text(encoding="utf-8"))


def match_person(people, login=None, name=None, email=None):
    """Team id for a GitHub login or a git author name/email, else None."""
    if email:
        m = re.match(r"^(?:\d+\+)?([^@]+)@users\.noreply\.github\.com$", email, re.I)
        if m:
            login = login or m.group(1)
    if login:
        for p in people:
            if p["github"] and p["github"].lower() == login.lower():
                return p["id"]
    if name:
        n = name.strip().lower()
        first = n.split()[0] if n.split() else ""
        for p in people:
            if n in (p["github"].lower(), p["name"].lower(), p["id"]):
                return p["id"]
        for p in people:
            if first and first == p["name"].lower():
                return p["id"]
    return None


# ---------------------------------------------------------------- git

def matches(path, patterns):
    return any(glob_to_regex(p).match(path) for p in patterns)


def pages_for_files(files, pages):
    """Pages whose own file or code globs cover any of these repo paths."""
    hit = []
    for pid, page in pages.items():
        own = rel(page["file"])
        if any(f == own or (page["code"] and matches(f, page["code"])) for f in files):
            hit.append(pid)
    return sorted(hit)


def git_commits(limit=3000, all_branches=False):
    """Commits newest first: {sha,date,author,email,message,files}.

    all_branches adds every remote branch, so work that hasn't reached master
    yet (a teammate's feature branch) still counts as something they worked on.
    """
    refs = ["HEAD", "--remotes"] if all_branches else []
    r = subprocess.run(
        ["git", "log", *refs, f"-{limit}", "--no-merges", "--format=%x1e%H%x1f%aI%x1f%an%x1f%ae%x1f%s", "--name-only"],
        cwd=ROOT, capture_output=True, text=True)
    if r.returncode != 0:
        warn("git log failed; building without history.")
        return []
    commits = []
    for chunk in r.stdout.split("\x1e"):
        if not chunk.strip():
            continue
        head, _, rest = chunk.partition("\n")
        sha, date, author, email, message = head.split("\x1f", 4)
        files = [f for f in rest.splitlines() if f.strip()]
        commits.append({"sha": sha, "date": date, "author": author, "email": email,
                        "message": message, "files": files})
    return commits


def history(commits, pages, people):
    """Return (last_change by page, recent list)."""
    last, recent = {}, []
    for c in commits:
        touched = pages_for_files(c["files"], pages)
        if not touched:
            continue
        entry = {"sha": c["sha"], "date": c["date"], "author": c["author"],
                 "person": match_person(people, name=c["author"], email=c["email"]),
                 "message": c["message"]}
        for pid in touched:
            last.setdefault(pid, entry)
        if len(recent) < RECENT_LIMIT:
            recent.append({**entry, "pages": touched})
    return last, recent


# Only program code counts as "worked on", not docs, configs at the root or Docker.
CODE_ROOTS = ("src/", "host_tools/")
BULK_COMMIT_FILES = 60


def code_path(f, packages):
    """Map a pre-src/ path (go2_llm_agent/x.py) to today's layout (src/go2_llm_agent/x.py).

    Packages moved under src/ on 2026-10-20; older branches still use the
    root layout, and their work should land on the same pages. `packages` is
    the set of src/ folder names the wiki pages claim.
    """
    if "/" in f and not f.startswith(CODE_ROOTS) and f.split("/", 1)[0] in packages:
        return "src/" + f
    return f


def contributions(commits, pages, people, limit=8):
    """Pages each person has worked on, from git: {person id: [slug, ...]}.

    A commit counts for a page when it touches program code (CODE_ROOTS) that
    the page's code: paths cover. Docs, wiki edits and bulk moves don't count:
    writing about a program or moving folders isn't working on it. Pages are sorted by that person's commit count (ties by slug), top
    `limit`. The center page (README.md) is left out: everyone edits it.
    """
    counts = {p["id"]: {} for p in people}
    packages = {g.split("/")[1] for page in pages.values() for g in page["code"]
                if g.startswith("src/") and g.count("/") >= 1}
    for c in commits:
        pid = match_person(people, name=c["author"], email=c["email"])
        if pid is None:
            continue
        if len(c["files"]) > BULK_COMMIT_FILES:
            continue  # a repo-wide move or cleanup says nothing about who works on what
        code_files = [code_path(f, packages) for f in c["files"]]
        code_files = [f for f in code_files if f.startswith(CODE_ROOTS)]
        for page in pages_for_files(code_files, pages):
            if page != CENTER:
                counts[pid][page] = counts[pid].get(page, 0) + 1
    return {pid: [s for s, _ in sorted(cs.items(), key=lambda kv: (-kv[1], kv[0]))[:limit]]
            for pid, cs in counts.items()}


# ---------------------------------------------------------------- branches

def parse_date(iso):
    return datetime.fromisoformat(iso.replace("Z", "+00:00"))


def active_branches(refs, pages, people, work, now, info, days=BRANCH_DAYS):
    """Sandbox branches people pushed lately, newest first.

    refs: [{name, sha, date, author, email, message}] for each remote branch
    tip (date is the tip's commit date). Branches whose tip is older than
    `days`, and branches with nothing master lacks (already merged), are left
    out. info(name) -> (ahead, files): commits not on master, and the files
    changed since the branch left master. Files in the pre-src/ layout land
    on today's pages, as in contributions().
    """
    cutoff = now - timedelta(days=days)
    packages = {g.split("/")[1] for page in pages.values() for g in page["code"]
                if g.startswith("src/") and g.count("/") >= 1}
    prs = {w["branch"]: w for w in work if w.get("kind") == "pr" and w.get("branch")}
    out = []
    for ref in refs:
        if parse_date(ref["date"]) < cutoff:
            continue
        ahead, files = info(ref["name"])
        if not ahead:
            continue
        pr = prs.get(ref["name"])
        out.append({
            "name": ref["name"],
            "person": match_person(people, name=ref["author"], email=ref["email"]),
            "author": ref["author"],
            "last_commit": {"sha": ref["sha"], "date": ref["date"], "message": ref["message"]},
            "ahead": ahead,
            "pages": pages_for_files([code_path(f, packages) for f in files], pages),
            "pr": {"number": pr["number"], "url": pr["url"]} if pr else None,
        })
    out.sort(key=lambda b: parse_date(b["last_commit"]["date"]), reverse=True)
    return out


def git_branch_refs(base=BASE_REF):
    """Tips of the origin branches other than the base: list of ref dicts."""
    fmt = "%(refname)%1f%(objectname)%1f%(committerdate:iso-strict)%1f%(authorname)%1f%(authoremail)%1f%(subject)"
    r = subprocess.run(["git", "for-each-ref", "refs/remotes/origin", f"--format={fmt}"],
                       cwd=ROOT, capture_output=True, text=True)
    if r.returncode != 0:
        warn("git for-each-ref failed; building without active branches.")
        return []
    refs = []
    for line in r.stdout.splitlines():
        parts = line.split("\x1f", 5)
        if len(parts) != 6:
            continue
        refname, sha, date, author, email, message = parts
        if refname == base or refname.endswith("/HEAD"):
            continue
        refs.append({"name": refname[len("refs/remotes/origin/"):], "sha": sha, "date": date,
                     "author": author, "email": email.strip("<>"), "message": message})
    return refs


def git_branch_info(name, base=BASE_REF):
    """(commits not on the base, files changed since the merge base)."""
    ref = f"refs/remotes/origin/{name}"
    count = subprocess.run(["git", "rev-list", "--count", f"{base}..{ref}"], cwd=ROOT, capture_output=True, text=True)
    diff = subprocess.run(["git", "diff", "--name-only", f"{base}...{ref}"], cwd=ROOT, capture_output=True, text=True)
    if count.returncode != 0 or diff.returncode != 0:
        return 0, []
    return int(count.stdout.strip() or 0), [f for f in diff.stdout.splitlines() if f.strip()]


def fetch_branches(pages, people, work):
    if subprocess.run(["git", "rev-parse", "--verify", "--quiet", BASE_REF], cwd=ROOT,
                      capture_output=True).returncode != 0:
        warn(f"no {BASE_REF}; building without active branches.")
        return []
    return active_branches(git_branch_refs(), pages, people, work, datetime.now(timezone.utc), git_branch_info)


# ---------------------------------------------------------------- GitHub

def wiki_pages_from_labels(labels, pages):
    out = []
    for label in labels:
        name = label.get("name", "") if isinstance(label, dict) else label
        if name.startswith("wiki:") and name[5:] in pages:
            out.append(name[5:])
    return sorted(set(out))


def label_names(item):
    return {(lb.get("name", "") if isinstance(lb, dict) else lb) for lb in item.get("labels", [])}


def question_from_issue(issue, comments, pages, excerpt=280):
    """A question record, or None if the issue is not a question on a page."""
    names = label_names(issue)
    if "question" not in names:
        return None
    on = wiki_pages_from_labels(issue.get("labels", []), pages)
    if not on:
        return None
    asker = (issue.get("user") or {}).get("login", "")
    answer = next((c for c in comments
                   if (c.get("user") or {}).get("login", "") != asker
                   and not (c.get("user") or {}).get("login", "").endswith("[bot]")), None)
    text = " ".join((answer or {}).get("body", "").split())
    if len(text) > excerpt:
        text = text[:excerpt - 3].rstrip() + "..."
    return {
        "number": issue["number"],
        "title": issue.get("title", ""),
        "url": issue.get("html_url", ""),
        "page": on[0],
        "asker": asker,
        "answered": answer is not None or "answered" in names,
        "answer_excerpt": text,
        "updated_at": issue.get("updated_at", ""),
    }


def work_from_issue(issue, pages, people):
    on = wiki_pages_from_labels(issue.get("labels", []), pages)
    if not on or "question" in label_names(issue):
        return None
    author = (issue.get("user") or {}).get("login", "")
    return {
        "kind": "issue", "number": issue["number"], "title": issue.get("title", ""),
        "url": issue.get("html_url", ""), "author": author,
        "person": match_person(people, login=author),
        "assignees": _assignees(issue, people),
        "pages": on, "review_requested": False, "draft": False,
        "updated_at": issue.get("updated_at", ""),
    }


def work_from_pr(pr, files, pages, people, repo=None):
    author = (pr.get("user") or {}).get("login", "")
    head = pr.get("head") or {}
    head_repo = ((head.get("repo") or {}).get("full_name") or "").lower()
    # Only a branch in this repo can match an active branch; a fork's "main" is not ours.
    branch = head.get("ref") if head.get("ref") and (repo is None or head_repo == repo.lower()) else None
    on = sorted(set(pages_for_files(files, pages)) | set(wiki_pages_from_labels(pr.get("labels", []), pages)))
    review = bool(pr.get("requested_reviewers") or pr.get("requested_teams") or "review" in label_names(pr))
    return {
        "kind": "pr", "number": pr["number"], "title": pr.get("title", ""),
        "url": pr.get("html_url", ""), "author": author,
        "person": match_person(people, login=author),
        "assignees": _assignees(pr, people),
        "pages": on, "review_requested": review, "draft": bool(pr.get("draft")),
        "branch": branch,
        "updated_at": pr.get("updated_at", ""),
    }


def _assignees(item, people):
    ids = []
    for a in item.get("assignees") or []:
        pid = match_person(people, login=a.get("login"))
        if pid and pid not in ids:
            ids.append(pid)
    return ids


class GitHub:
    def __init__(self, token, repo):
        self.token, self.repo = token, repo

    def get_all(self, path):
        url = f"https://api.github.com/repos/{self.repo}/{path}"
        out = []
        while url:
            req = urllib.request.Request(url, headers={
                "Authorization": f"Bearer {self.token}",
                "Accept": "application/vnd.github+json",
                "User-Agent": "oak-wiki-build",
            })
            with urllib.request.urlopen(req, timeout=20) as resp:
                out.extend(json.load(resp))
                link = resp.headers.get("Link", "")
            m = re.search(r'<([^>]+)>;\s*rel="next"', link)
            url = m.group(1) if m else None
        return out


def fetch_github(pages, people):
    """Return (work, questions). Empty lists without a token or on errors."""
    token = os.environ.get("GITHUB_TOKEN")
    repo = os.environ.get("GITHUB_REPOSITORY")
    if not token or not repo:
        return [], []
    gh = GitHub(token, repo)
    work, questions = [], []
    try:
        for issue in gh.get_all("issues?state=open&per_page=100"):
            if "pull_request" in issue:
                continue
            w = work_from_issue(issue, pages, people)
            if w:
                work.append(w)
        for pr in gh.get_all("pulls?state=open&per_page=100"):
            files = [f["filename"] for f in gh.get_all(f"pulls/{pr['number']}/files?per_page=100")]
            work.append(work_from_pr(pr, files, pages, people, repo))
        for issue in gh.get_all("issues?state=all&labels=question&per_page=100"):
            if "pull_request" in issue:
                continue
            comments = gh.get_all(f"issues/{issue['number']}/comments?per_page=100") if issue.get("comments") else []
            q = question_from_issue(issue, comments, pages)
            # A closed question nobody answered is dropped, not shown forever.
            if q and (issue.get("state") == "open" or q["answered"]):
                questions.append(q)
    except (urllib.error.URLError, OSError, ValueError, KeyError) as err:
        warn(f"could not read GitHub ({err}); building without work and questions.")
        return [], []
    work.sort(key=lambda w: w["updated_at"], reverse=True)
    questions.sort(key=lambda q: q["updated_at"], reverse=True)
    return work, questions


# ---------------------------------------------------------------- main

def github_branch():
    """Branch the "View on GitHub" links point at, and whether GitHub has it yet.

    WIKI_BRANCH wins (the Pages workflow sets it to the default branch, since
    it also runs on issue and pull request events). Then GITHUB_REF_NAME on a
    push. Locally it is the current branch, and links stay off until that
    branch exists on origin, so a preview never sends people to a 404.
    """
    if os.environ.get("WIKI_BRANCH"):
        return os.environ["WIKI_BRANCH"], True
    if os.environ.get("GITHUB_REF_NAME"):
        return os.environ["GITHUB_REF_NAME"], True

    def git(*args):
        r = subprocess.run(["git", *args], cwd=ROOT, capture_output=True, text=True)
        return r.stdout.strip() if r.returncode == 0 else ""

    branch = git("rev-parse", "--abbrev-ref", "HEAD") or "master"
    return branch, bool(git("rev-parse", "--verify", "--quiet", f"refs/remotes/origin/{branch}"))


def build():
    if not WIKI.is_dir():
        fail(f"no wiki folder at {rel(WIKI)}.")
    pages = load_pages()
    edges, backlinks = collect_edges(pages)
    people = load_team()
    commits = git_commits()
    last, recent = history(commits, pages, people)
    contributed = contributions(git_commits(all_branches=True), pages, people)
    for p in people:
        unknown = [s for s in p["past"] if s not in pages]
        if unknown:
            warn(f"{rel(TEAM)}: {p['id']} has past pages that do not exist: {', '.join(unknown)}.")
        p["past"] = [s for s in p["past"] if s in pages]
        p["contributed"] = contributed.get(p["id"], [])
        p["worked_on"] = list(dict.fromkeys(p["past"] + p["contributed"]))
    work, questions = fetch_github(pages, people)
    branches = fetch_branches(pages, people, work)

    nodes = []
    for pid, page in pages.items():
        hub, depth = hub_and_depth(pid, pages)
        nodes.append({
            "id": pid,
            "title": page["title"],
            "parent": page["parent"],
            "hub": hub,
            "depth": depth,
            "code": page["code"],
            "path": rel(page["file"]),
            "markdown": page["markdown"],
            "summary": summarize(page["markdown"]),
            "x": 0.0,
            "y": 0.0,
            "backlinks": backlinks.get(pid, []),
            "last_change": last.get(pid),
        })
    positions = layout(nodes)
    for n in nodes:
        n["x"], n["y"] = positions[n["id"]]

    branch, published = github_branch()
    return {
        "version": 2,
        "branch": branch,
        "base": BASE_REF.rsplit("/", 1)[-1],  # the main branch new work starts from
        "published": published,
        "generated_at": datetime.now(timezone.utc).replace(microsecond=0).isoformat(),
        "repo": os.environ.get("GITHUB_REPOSITORY") or DEFAULT_REPO,
        "people": people,
        "nodes": nodes,
        "edges": edges,
        "work": work,
        "questions": questions,
        "recent": recent,
        "branches": branches,
    }


def main():
    graph = build()
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(graph, indent=1), encoding="utf-8")
    # The same data as a script, for viewers that block fetch(): a sandboxed
    # frame (Lavish review) or opening index.html straight from disk.
    OUT.with_suffix(".js").write_text("window.OAK_GRAPH = " + json.dumps(graph) + ";\n", encoding="utf-8")
    links = sum(1 for e in graph["edges"] if e["kind"] == "link")
    prs = sum(1 for w in graph["work"] if w["kind"] == "pr")
    print(f"Wrote {rel(OUT)}: {len(graph['nodes'])} pages, {len(graph['edges']) - links} parent edges, "
          f"{links} cross links, {len(graph['people'])} people, {len(graph['recent'])} recent commits, "
          f"{len(graph['work']) - prs} work issues, {prs} open PRs, {len(graph['questions'])} questions, "
          f"{len(graph['branches'])} active branches.")


if __name__ == "__main__":
    main()
