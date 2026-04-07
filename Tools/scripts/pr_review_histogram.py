#!/usr/bin/env python3
"""
Histogram of GitHub PR inline review comment counts by author.

Usage:
  python pr_review_histogram.py [options]

Options:
  --repo OWNER/REPO     Repository to query (default: ArduPilot/ardupilot)
  --since YYYY-MM-DD    Only count comments on or after this date (default: 2 years ago)
  --no-bots             Exclude accounts whose login ends with [bot]
  --save FILE.png       Save the chart to a file instead of displaying it
  --table               Print a plain-text table instead of a chart
  --top N               Show top N authors in chart (default: 30)
  --refresh             Clear cached pages and re-fetch from GitHub

By default, full pages (100 items) from a previous run are reused, and only
the last (potentially incomplete) page and any new pages beyond it are fetched.
"""

import argparse
import json
import subprocess
import sys
from collections import Counter
from datetime import datetime, timedelta, timezone
from pathlib import Path


CACHE_DIR = Path.home() / ".cache" / "pr_review_histogram"


def get_token():
    result = subprocess.run(
        ["gh", "auth", "token"],
        capture_output=True, text=True, check=True
    )
    return result.stdout.strip()


def cache_path(repo, since_str, page):
    repo_safe = repo.replace("/", "_")
    return CACHE_DIR / f"{repo_safe}_{since_str}_page{page:05d}.json"


def find_last_cached_page(repo, since_str):
    """Return the highest page number that has a cache file, or 0 if none."""
    repo_safe = repo.replace("/", "_")
    prefix = f"{repo_safe}_{since_str}_page"
    pages = []
    for p in CACHE_DIR.glob(f"{prefix}*.json"):
        try:
            pages.append(int(p.stem[len(prefix):]))
        except ValueError:
            pass
    return max(pages) if pages else 0


def _do_request(session, url, params, page):
    import time
    import requests
    p = dict(params)
    p["page"] = page
    for attempt in range(5):
        try:
            resp = session.get(url, params=p, timeout=30)
            if resp.status_code in (502, 503, 504):
                raise requests.exceptions.ConnectionError(f"HTTP {resp.status_code}")
            resp.raise_for_status()
            return resp.json()
        except (requests.exceptions.ChunkedEncodingError,
                requests.exceptions.ConnectionError) as e:
            wait = 2 ** attempt
            print(f"\n  {e}, retrying in {wait}s...", file=sys.stderr)
            time.sleep(wait)
    raise RuntimeError(f"Failed to fetch page {page} after 5 attempts")


def fetch_all_comments(repo, since_dt, refresh):
    try:
        import requests
    except ImportError:
        sys.exit("The 'requests' package is required: pip install requests")

    token = get_token()
    session = requests.Session()
    session.headers.update({
        "Authorization": f"Bearer {token}",
        "Accept": "application/vnd.github+json",
        "X-GitHub-Api-Version": "2022-11-28",
    })

    url = f"https://api.github.com/repos/{repo}/pulls/comments"
    since_str = since_dt.strftime("%Y-%m-%d")
    params = {
        "per_page": 100,
        "sort": "created",
        "direction": "asc",
        "since": since_dt.strftime("%Y-%m-%dT%H:%M:%SZ"),
    }

    CACHE_DIR.mkdir(parents=True, exist_ok=True)

    if refresh:
        # Wipe existing cache entries for this repo+since
        for p in CACHE_DIR.glob(f"{repo.replace('/', '_')}_{since_str}_page*.json"):
            p.unlink()
        start_page = 1
    else:
        last = find_last_cached_page(repo, since_str)
        if last > 1:
            # The last cached page may have been incomplete; re-fetch it.
            cache_path(repo, since_str, last).unlink()
            start_page = last
        else:
            start_page = 1

    # Load fully-cached pages before the start_page
    comments = []
    for p in range(1, start_page):
        with open(cache_path(repo, since_str, p)) as f:
            comments.extend(json.load(f))

    if start_page > 1:
        print(f"Resuming from page {start_page} ({len(comments)} comments cached).", file=sys.stderr)

    page = start_page
    while True:
        cp = cache_path(repo, since_str, page)
        if cp.exists():
            with open(cp) as f:
                data = json.load(f)
        else:
            print(f"\rFetching page {page}... ({len(comments)} comments so far)", end="", flush=True, file=sys.stderr)
            data = _do_request(session, url, params, page)
            with open(cp, "w") as f:
                json.dump(data, f)

        if not data:
            break
        comments.extend(data)
        if len(data) < 100:
            break
        page += 1

    print(f"\rFetched {len(comments)} comments across {page} pages.          ", file=sys.stderr)
    return comments


def build_counts(comments, no_bots):
    counts = Counter()
    for c in comments:
        user = c.get("user") or {}
        login = user.get("login", "unknown")
        if no_bots and login.endswith("[bot]"):
            continue
        counts[login] += 1
    return counts


def print_table(counts, top):
    rows = counts.most_common(top)
    max_login = max(len(r[0]) for r in rows)
    print(f"{'Author':<{max_login}}  {'Comments':>8}")
    print("-" * (max_login + 10))
    for login, count in rows:
        print(f"{login:<{max_login}}  {count:>8}")


def plot_chart(counts, top, since_dt, repo, save_path):
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        sys.exit("The 'matplotlib' package is required: pip install matplotlib")

    rows = counts.most_common(top)
    rows.reverse()  # highest at top
    labels = [r[0] for r in rows]
    values = [r[1] for r in rows]

    fig, ax = plt.subplots(figsize=(10, max(6, len(rows) * 0.35)))
    bars = ax.barh(labels, values, color="steelblue")
    ax.bar_label(bars, padding=3, fontsize=8)
    ax.set_xlabel("Inline review comments")
    since_label = since_dt.strftime("%Y-%m-%d")
    ax.set_title(f"{repo} — PR inline review comments by author\n(since {since_label}, top {top})")
    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150)
        print(f"Saved to {save_path}", file=sys.stderr)
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--repo", default="ArduPilot/ardupilot", metavar="OWNER/REPO")
    parser.add_argument("--since", metavar="YYYY-MM-DD",
                        default=(datetime.now(timezone.utc) - timedelta(days=730)).strftime("%Y-%m-%d"))
    parser.add_argument("--no-bots", action="store_true")
    parser.add_argument("--save", metavar="FILE.png")
    parser.add_argument("--table", action="store_true")
    parser.add_argument("--top", type=int, default=30, metavar="N")
    parser.add_argument("--refresh", action="store_true")
    args = parser.parse_args()

    since_dt = datetime.strptime(args.since, "%Y-%m-%d").replace(tzinfo=timezone.utc)

    comments = fetch_all_comments(args.repo, since_dt, args.refresh)
    counts = build_counts(comments, args.no_bots)

    if not counts:
        print("No comments found.", file=sys.stderr)
        sys.exit(0)

    print(f"Total authors: {len(counts)}, total comments: {sum(counts.values())}", file=sys.stderr)

    if args.table:
        print_table(counts, args.top)
    else:
        plot_chart(counts, args.top, since_dt, args.repo, args.save)


if __name__ == "__main__":
    main()
