#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Analyse the GitHub Actions queue for a repository or organisation.

Fetches all runs (paginated) and writes a report broken down by PR and author.

NOTE ON TERMINOLOGY
  A workflow *run* is one execution of one .yml workflow file.
  Each run contains one or more *jobs*.  This script counts runs by default
  because fetching per-job counts requires one extra API call per run and is
  slow.  Use --jobs to enable that (adds ~1 API call per queued run).

  The GitHub PR page shows individual job statuses (check-runs), so its
  queued count will be higher than the run count shown here.

Usage:
    gha_queue_analysis.py [--org ORG | --repo OWNER/REPO] [--out FILE]
                          [--status STATUS] [--jobs]

Defaults:
    --org    ArduPilot      (scan all non-archived repos in the org)
    --out    /tmp/gha_queue_report_YYYYMMDD_HHMMSS.txt
    --status queued        (use 'all' for every status)

Requires: gh CLI authenticated with repo read access.
"""

import argparse
import json
import re
import subprocess
import sys
import time
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path


# ---------------------------------------------------------------------------
# GitHub API helpers
# ---------------------------------------------------------------------------

def gh_api_paginate(endpoint, extra_params=""):
    """Return a list of all items from a paginated gh api call."""
    if extra_params:
        url = f"{endpoint}?{extra_params}&per_page=100"
    else:
        url = f"{endpoint}?per_page=100"

    result = subprocess.run(
        ["gh", "api", "--paginate", url],
        capture_output=True, text=True,
    )
    if result.returncode != 0:
        print(f"gh api error: {result.stderr.strip()}", file=sys.stderr)
        sys.exit(1)

    # --paginate emits one JSON object per page; combine arrays.
    items = []
    decoder = json.JSONDecoder()
    pos = 0
    text = result.stdout.strip()
    while pos < len(text):
        obj, end = decoder.raw_decode(text, pos)
        if isinstance(obj, dict):
            for key in ("workflow_runs", "jobs", "items"):
                if key in obj:
                    items.extend(obj[key])
                    break
            else:
                items.append(obj)
        elif isinstance(obj, list):
            items.extend(obj)
        pos = end
        while pos < len(text) and text[pos] in " \t\n\r":
            pos += 1
    return items


def gh_api_single(endpoint):
    """Fetch a single (non-paginated) API endpoint, return parsed JSON."""
    result = subprocess.run(
        ["gh", "api", endpoint],
        capture_output=True, text=True,
    )
    if result.returncode != 0:
        return None
    return json.loads(result.stdout)


def fetch_repos_in_org(org):
    """Return list of 'owner/repo' full_names for all non-archived repos in org."""
    repos = gh_api_paginate(f"orgs/{org}/repos", "type=all")
    return [r["full_name"] for r in repos if not r.get("archived")]


def fetch_runs(repo, status):
    """Fetch workflow runs, optionally filtered by status.

    When status is 'queued' we also fetch 'in_progress' runs so that the
    running/failed/ok columns in the report are populated.  Fetching all
    completed runs is too expensive by default; use status='all' for that.

    Each returned run dict has a '_repo' field set to repo.
    """
    endpoint = f"repos/{repo}/actions/runs"
    if status == "all":
        runs = gh_api_paginate(endpoint, "per_page=100")
    elif status == "queued":
        # Fetch both queued and in_progress to give a complete live picture.
        runs = []
        for s in ("queued", "in_progress"):
            runs.extend(gh_api_paginate(endpoint, f"status={s}&per_page=100"))
    else:
        runs = gh_api_paginate(endpoint, f"status={status}&per_page=100")
    for run in runs:
        run["_repo"] = repo
    return runs


def fetch_runs_for_repos(repos, status):
    """Fetch runs from every repo in the list, returning all combined."""
    all_runs = []
    for repo in repos:
        t0 = time.perf_counter()
        repo_runs = fetch_runs(repo, status)
        elapsed = time.perf_counter() - t0
        if repo_runs:
            print(f"  {repo}: {len(repo_runs)} runs in {elapsed:.1f}s",
                  file=sys.stderr)
        else:
            print(f"  {repo}: 0 runs in {elapsed:.1f}s", file=sys.stderr)
        all_runs.extend(repo_runs)
    return all_runs


def fetch_prs(repo):
    """Return dict mapping head_branch -> PR info, for all open PRs."""
    prs = gh_api_paginate(f"repos/{repo}/pulls", "state=open")
    by_branch = {}
    for pr in prs:
        branch = pr["head"]["ref"]
        # If two forks both have the same branch name, keep the one whose
        # head repo matches the target repo (i.e. not a fork), else first seen.
        head_repo = (pr["head"]["repo"] or {}).get("full_name", "")
        base_repo = (pr["base"]["repo"] or {}).get("full_name", "")
        this_fork = bool(head_repo and base_repo and head_repo != base_repo)
        if branch in by_branch:
            existing_fork = by_branch[branch].get("_fork", True)
            if existing_fork and not this_fork:
                pass  # replace with non-fork version
            else:
                continue
        by_branch[branch] = {
            "number": pr["number"],
            "author": pr["user"]["login"],
            "draft": pr["draft"],
            "title": pr["title"],
            "created": pr["created_at"],
            "_fork": this_fork,
        }
    return by_branch


def fetch_run_job_counts(repo, run_id):
    """Return (queued, in_progress, success, failure, cancelled) job counts."""
    jobs = gh_api_paginate(f"repos/{repo}/actions/runs/{run_id}/jobs")
    q = sum(1 for j in jobs if j["status"] == "queued")
    r = sum(1 for j in jobs if j["status"] == "in_progress")
    ok = sum(1 for j in jobs if j.get("conclusion") == "success")
    fl = sum(1 for j in jobs if j.get("conclusion") == "failure")
    ca = sum(1 for j in jobs if j.get("conclusion") == "cancelled")
    return q, r, ok, fl, ca


# ---------------------------------------------------------------------------
# Time helpers
# ---------------------------------------------------------------------------

NOW = datetime.now(timezone.utc)


def parse_ts(ts):
    if not ts:
        return None
    return datetime.fromisoformat(ts.replace("Z", "+00:00"))


def age_str(ts):
    """Human-readable age from an ISO timestamp string."""
    dt = parse_ts(ts)
    if dt is None:
        return "?"
    secs = int((NOW - dt).total_seconds())
    if secs < 60:
        return f"{secs}s"
    if secs < 3600:
        return f"{secs // 60}m"
    h, m = divmod(secs // 60, 60)
    return f"{h}h{m:02d}m"


# ---------------------------------------------------------------------------
# PR attribution
# ---------------------------------------------------------------------------

_PULL_REF_RE = re.compile(r"^refs/pull/(\d+)/")


def pr_from_run(run, pr_maps):
    """Return PR info dict for a run, or None.

    pr_maps is a dict keyed by 'owner/repo', each value being the per-repo
    branch -> PR info map returned by fetch_prs().

    Resolution order:
      1. pull_requests[] array embedded in the run (most reliable)
      2. refs/pull/NNN/head branch name
      3. Branch name lookup in pr_map
    """
    repo = run.get("_repo", "")
    pr_map = pr_maps.get(repo, {})

    # 1. Embedded pull_requests
    prs = run.get("pull_requests") or []
    if prs:
        pr = prs[0]
        branch = run["head_branch"]
        return pr_map.get(branch) or {
            "number": pr["number"],
            "author": run.get("actor", {}).get("login", "?"),
            "draft": False,
            "title": run.get("display_title", ""),
            "created": run.get("created_at", ""),
        }

    # 2. refs/pull/NNN/head branch name
    m = _PULL_REF_RE.match(run.get("head_branch", ""))
    if m:
        pr_num = int(m.group(1))
        # Try to find in pr_map by number
        for info in pr_map.values():
            if info["number"] == pr_num:
                return info
        return {
            "number": pr_num,
            "author": run.get("actor", {}).get("login", "?"),
            "draft": False,
            "title": run.get("display_title", ""),
            "created": run.get("created_at", ""),
        }

    # 3. Branch name lookup
    branch = run.get("head_branch", "")
    return pr_map.get(branch)


# ---------------------------------------------------------------------------
# Analysis
# ---------------------------------------------------------------------------

def analyse(runs, pr_maps, fetch_jobs):
    """Return (by_branch, by_author) analysis dicts."""
    by_branch = defaultdict(lambda: {
        "runs": [], "title": "", "actor": "", "pr": None, "repo": "",
        "branch": "",
    })

    for r in runs:
        repo = r.get("_repo", "")
        branch = r["head_branch"]
        key = f"{repo}#{branch}"
        b = by_branch[key]
        b["repo"] = repo
        b["branch"] = branch
        b["title"] = r.get("display_title") or r.get("name", "")
        b["actor"] = r.get("actor", {}).get("login", "?")

        pr = pr_from_run(r, pr_maps)
        if pr and b["pr"] is None:
            b["pr"] = pr

        run_entry = {
            "name": r["name"],
            "status": r["status"],
            "conclusion": r.get("conclusion") or "",
            "created": r.get("created_at", ""),
            "updated": r.get("updated_at", ""),
            "run_id": r["id"],
            "_repo": repo,
            # Job-level counts populated below if --jobs
            "job_counts": None,
        }

        if fetch_jobs and r["status"] in ("queued", "in_progress"):
            print(f"  fetching jobs for run {r['id']} ({r['name']})…",
                  file=sys.stderr)
            run_entry["job_counts"] = fetch_run_job_counts(repo, r["id"])

        b["runs"].append(run_entry)

    # Per-author rollup
    by_author = defaultdict(lambda: {
        "branches": [], "queued": 0, "running": 0,
        "failed": 0, "ok": 0, "cancelled": 0,
        "jobs_queued": 0,
    })
    for key, info in by_branch.items():
        author = info["pr"]["author"] if info["pr"] else info["actor"]
        a = by_author[author]
        if key not in a["branches"]:
            a["branches"].append(key)
        for run in info["runs"]:
            if run["status"] == "queued":
                a["queued"] += 1
            elif run["status"] == "in_progress":
                a["running"] += 1
            elif run["conclusion"] == "success":
                a["ok"] += 1
            elif run["conclusion"] == "failure":
                a["failed"] += 1
            elif run["conclusion"] == "cancelled":
                a["cancelled"] += 1
            if run["job_counts"]:
                a["jobs_queued"] += run["job_counts"][0]

    return by_branch, by_author


def oldest_queued(runs):
    times = [r["created"] for r in runs if r["status"] == "queued" and r["created"]]
    return min(times) if times else None


# ---------------------------------------------------------------------------
# Report rendering
# ---------------------------------------------------------------------------

def render(runs, by_branch, by_author, label, status_filter, out_path, fetch_jobs):
    lines = []
    w = lines.append

    total_q = sum(1 for r in runs if r["status"] == "queued")
    total_r = sum(1 for r in runs if r["status"] == "in_progress")
    total_ok = sum(1 for r in runs if r.get("conclusion") == "success")
    total_fail = sum(1 for r in runs if r.get("conclusion") == "failure")
    total_cancel = sum(1 for r in runs if r.get("conclusion") == "cancelled")

    w("=" * 76)
    w(f"GitHub Actions Queue Analysis — {label}")
    w(f"Generated : {NOW.strftime('%Y-%m-%d %H:%M:%S UTC')}")
    w(f"Filter    : status={status_filter}")
    w("=" * 76)
    w("")
    w("NOTE: counts are workflow *runs*, not individual jobs.")
    w("      Each run typically contains several jobs.")
    w("      The PR page shows job counts; this report shows run counts.")
    if fetch_jobs:
        w("      --jobs: queued job counts fetched and shown where available.")
    w("")

    # List repos with activity when in org mode (label contains " (all repos)")
    active_repos = sorted({r.get("_repo", "") for r in runs if r.get("_repo")})
    if len(active_repos) > 1:
        w(f"Repos with activity ({len(active_repos)}):")
        for rp in active_repos:
            repo_q = sum(1 for r in runs if r.get("_repo") == rp and r["status"] == "queued")
            repo_r = sum(1 for r in runs if r.get("_repo") == rp and r["status"] == "in_progress")
            w(f"  {rp:<45}  queued={repo_q}  running={repo_r}")
        w("")

    w(f"Total workflow runs : {len(runs)}")
    w(f"  Queued            : {total_q}")
    w(f"  In progress       : {total_r}")
    w(f"  Succeeded         : {total_ok}")
    w(f"  Failed            : {total_fail}")
    w(f"  Cancelled         : {total_cancel}")
    w(f"Branches / PRs      : {len(by_branch)}")
    w("")

    # ---- Per-branch ----
    w("=" * 76)
    w("PER BRANCH / PR")
    w("=" * 76)

    def sort_key(item):
        branch_runs = item[1]["runs"]
        has_fail = any(r["conclusion"] == "failure" for r in branch_runs)
        has_running = any(r["status"] == "in_progress" for r in branch_runs)
        has_queued = any(r["status"] == "queued" for r in branch_runs)
        oldest = oldest_queued(branch_runs) or "9999"
        if has_fail:
            return (0, oldest)
        if has_running:
            return (1, oldest)
        if has_queued:
            return (2, oldest)
        return (3, oldest)

    for _key, info in sorted(by_branch.items(), key=sort_key):
        branch_runs = info["runs"]
        q  = [r for r in branch_runs if r["status"] == "queued"]
        rn = [r for r in branch_runs if r["status"] == "in_progress"]
        ok = [r for r in branch_runs if r["conclusion"] == "success"]
        fl = [r for r in branch_runs if r["conclusion"] == "failure"]
        ca = [r for r in branch_runs if r["conclusion"] == "cancelled"]

        pr = info["pr"]
        author = pr["author"] if pr else info["actor"]
        pr_num = f"#{pr['number']}" if pr else "(no open PR)"
        draft  = " [DRAFT]" if pr and pr["draft"] else ""

        w("")
        if info["repo"]:
            w(f"  Repo   : {info['repo']}")
        w(f"  Branch : {info['branch']}")
        w(f"  PR     : {pr_num}{draft}  Author: {author}")
        w(f"  Title  : {info['title'][:68]}")

        run_counts = (f"queued={len(q)}  running={len(rn)}  "
                      f"ok={len(ok)}  fail={len(fl)}  cancelled={len(ca)}")
        oldest = oldest_queued(branch_runs)
        wait = f"  oldest queued {age_str(oldest)} ago" if oldest else ""
        w(f"  Runs   : {len(branch_runs)} total  [{run_counts}]{wait}")

        # Job-level counts if fetched
        total_jobs_q = sum(
            r["job_counts"][0] for r in branch_runs if r["job_counts"]
        )
        if total_jobs_q:
            w(f"  Jobs   : ~{total_jobs_q} queued jobs across those runs")

        if fl:
            w(f"  FAILING: {', '.join(r['name'] for r in fl)}")
        if rn:
            w(f"  Running: {', '.join(r['name'] for r in rn)}")

    # ---- Per-author ----
    w("")
    w("=" * 76)
    w("PER AUTHOR  (sorted by queued run count)")
    w("=" * 76)
    w("")

    has_job_counts = any(a["jobs_queued"] for a in by_author.values())
    has_concluded = any(
        a["ok"] or a["failed"] or a["cancelled"] for a in by_author.values()
    )

    # Build column spec depending on what data is actually present.
    def _n_branches(a):
        return len(a["branches"])

    if has_job_counts and has_concluded:
        hdr = (f"  {'Author':<24} {'PRs':>4}  {'Q-runs':>6}  {'Q-jobs':>6}"
               f"  {'Running':>7}  {'OK':>4}  {'Fail':>4}  {'Cancelled':>9}")
        sep = (f"  {'-'*24}  {'-'*4}  {'-'*6}  {'-'*6}"
               f"  {'-'*7}  {'-'*4}  {'-'*4}  {'-'*9}")

        def row(author, a):
            return (f"  {author:<24} {_n_branches(a):>4}  {a['queued']:>6}"
                    f"  {a['jobs_queued']:>6}  {a['running']:>7}"
                    f"  {a['ok']:>4}  {a['failed']:>4}  {a['cancelled']:>9}")

    elif has_job_counts:
        hdr = (f"  {'Author':<24} {'PRs':>4}  {'Q-runs':>6}"
               f"  {'Q-jobs':>6}  {'Running':>7}")
        sep = f"  {'-'*24}  {'-'*4}  {'-'*6}  {'-'*6}  {'-'*7}"

        def row(author, a):
            return (f"  {author:<24} {_n_branches(a):>4}  {a['queued']:>6}"
                    f"  {a['jobs_queued']:>6}  {a['running']:>7}")

    elif has_concluded:
        hdr = (f"  {'Author':<24} {'PRs':>4}  {'Queued':>6}  {'Running':>7}"
               f"  {'OK':>4}  {'Fail':>4}  {'Cancelled':>9}")
        sep = (f"  {'-'*24}  {'-'*4}  {'-'*6}  {'-'*7}"
               f"  {'-'*4}  {'-'*4}  {'-'*9}")

        def row(author, a):
            return (f"  {author:<24} {_n_branches(a):>4}  {a['queued']:>6}"
                    f"  {a['running']:>7}  {a['ok']:>4}"
                    f"  {a['failed']:>4}  {a['cancelled']:>9}")

    else:
        hdr = f"  {'Author':<24} {'PRs':>4}  {'Queued':>6}  {'Running':>7}"
        sep = f"  {'-'*24}  {'-'*4}  {'-'*6}  {'-'*7}"

        def row(author, a):
            return (f"  {author:<24} {_n_branches(a):>4}"
                    f"  {a['queued']:>6}  {a['running']:>7}")

    w(hdr)
    w(sep)

    sorted_authors = sorted(by_author.items(),
                            key=lambda x: -(x[1]["queued"] + x[1]["running"]))
    for author, a in sorted_authors:
        w(row(author, a))

    # Totals row
    w(sep)
    totals = {
        "branches": list({b for a in by_author.values() for b in a["branches"]}),
        "queued": sum(a["queued"] for a in by_author.values()),
        "running": sum(a["running"] for a in by_author.values()),
        "ok": sum(a["ok"] for a in by_author.values()),
        "failed": sum(a["failed"] for a in by_author.values()),
        "cancelled": sum(a["cancelled"] for a in by_author.values()),
        "jobs_queued": sum(a["jobs_queued"] for a in by_author.values()),
    }
    w(row("TOTAL", totals))

    w("")
    w(f"Report written: {out_path}")
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--repo",   default=None,
                        help="GitHub repo in OWNER/REPO format "
                             "(mutually exclusive with --org)")
    parser.add_argument("--org",    default=None,
                        help="GitHub org — scan all non-archived repos "
                             "(mutually exclusive with --repo; default: ArduPilot)")
    parser.add_argument("--out",    default=None,
                        help="Output file path "
                             "(default: /tmp/gha_queue_report_YYYYMMDD_HHMMSS.txt)")
    parser.add_argument("--status", default="queued",
                        help="Run status filter: queued, in_progress, completed, all")
    parser.add_argument("--jobs",   action="store_true",
                        help="Fetch per-job counts for queued/in-progress runs "
                             "(one extra API call per run — slow)")
    args = parser.parse_args()

    if args.repo and args.org:
        parser.error("--repo and --org are mutually exclusive")

    if args.out is None:
        ts = NOW.strftime("%Y%m%d_%H%M%S")
        args.out = f"/tmp/gha_queue_report_{ts}.txt"

    script_t0 = time.perf_counter()

    if args.org or (not args.repo):
        # Org-wide mode (default)
        org = args.org or "ArduPilot"

        print(f"Listing repos in {org}…", file=sys.stderr)
        t0 = time.perf_counter()
        repos = fetch_repos_in_org(org)
        print(f"  {len(repos)} repos listed in {time.perf_counter()-t0:.1f}s",
              file=sys.stderr)

        print(f"Fetching runs from all repos (status={args.status})…",
              file=sys.stderr)
        t0 = time.perf_counter()
        runs = fetch_runs_for_repos(repos, args.status)
        print(f"  {len(runs)} total runs fetched in {time.perf_counter()-t0:.1f}s",
              file=sys.stderr)

        repos_with_runs = sorted({r["_repo"] for r in runs})
        print(f"Fetching open PRs for {len(repos_with_runs)} repos with runs…",
              file=sys.stderr)
        t0 = time.perf_counter()
        pr_maps = {}
        for repo in repos_with_runs:
            pr_maps[repo] = fetch_prs(repo)
        total_prs = sum(len(m) for m in pr_maps.values())
        print(f"  {total_prs} open PRs fetched in {time.perf_counter()-t0:.1f}s",
              file=sys.stderr)

        label = f"{org} (all repos)"
    else:
        # Single-repo mode
        repo = args.repo
        print(f"Fetching runs from {repo} (status={args.status})…",
              file=sys.stderr)
        t0 = time.perf_counter()
        runs = fetch_runs(repo, args.status)
        print(f"  {len(runs)} runs fetched in {time.perf_counter()-t0:.1f}s",
              file=sys.stderr)

        print("Fetching open PRs…", file=sys.stderr)
        t0 = time.perf_counter()
        pr_maps = {repo: fetch_prs(repo)}
        print(f"  {len(pr_maps[repo])} open PRs fetched in "
              f"{time.perf_counter()-t0:.1f}s", file=sys.stderr)

        label = repo

    by_branch, by_author = analyse(runs, pr_maps, args.jobs)
    report = render(runs, by_branch, by_author, label, args.status,
                    args.out, args.jobs)

    out = Path(args.out)
    out.write_text(report, encoding="utf-8")
    print(report)
    print(f"\nSaved to {out}", file=sys.stderr)
    print(f"Total elapsed: {time.perf_counter()-script_t0:.1f}s", file=sys.stderr)


if __name__ == "__main__":
    main()
