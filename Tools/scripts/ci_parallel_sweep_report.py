#!/usr/bin/env python3

'''
Summarise a Rover --parallel sweep run from the GitHub actions API.

The sweep workflow (.github/workflows/test_sitl_rover.yml on the
rover-parallel-sweep branch) runs the Rover autotest suite at a range of
--parallel values.  This pulls each leg's timing out of the API and draws
the time-taken-versus-parallelism curve.

We time the autotest *step* rather than the job, so runner setup, checkout
and ccache restore - which do not vary with --parallel - stay out of the
number.

AP_FLAKE8_CLEAN
'''

import argparse
import collections
import json
import re
import subprocess
import sys

from datetime import datetime


def gh_api(path):
    '''fetch path from the github API as decoded json'''
    cmd = ["gh", "api", "-H", "Accept: application/vnd.github+json", path]
    out = subprocess.run(cmd, check=True, capture_output=True, text=True).stdout
    return json.loads(out)


def parse_time(stamp):
    if stamp is None:
        return None
    return datetime.strptime(stamp, "%Y-%m-%dT%H:%M:%SZ")


def latest_run_id(repo, branch, workflow):
    '''id of the most recent run of workflow on branch'''
    runs = gh_api(f"/repos/{repo}/actions/workflows/{workflow}/runs?branch={branch}&per_page=1")
    if not runs["workflow_runs"]:
        raise ValueError(f"no runs of {workflow} on {branch}")
    return runs["workflow_runs"][0]["id"]


def fetch_jobs(repo, run_id):
    '''all jobs for run_id, following pagination'''
    jobs = []
    page = 1
    while True:
        got = gh_api(f"/repos/{repo}/actions/runs/{run_id}/jobs?per_page=100&page={page}")
        jobs.extend(got["jobs"])
        if len(jobs) >= got["total_count"]:
            break
        page += 1
    return jobs


# the sweep workflow names its legs "autotest (parallel=N repeat=M)"
NAME_RE = re.compile(r"parallel=(\d+) repeat=(\d+)")

Leg = collections.namedtuple("Leg", "parallel repeat seconds conclusion url")


def legs_from_jobs(jobs):
    '''pull one Leg out of each sweep job, skipping ones which never ran'''
    legs = []
    for job in jobs:
        matched = NAME_RE.search(job["name"])
        if matched is None:
            continue  # the build job, or something else
        step = None
        for candidate in job.get("steps", []):
            if candidate["name"].startswith("test rover at --parallel"):
                step = candidate
                break
        if step is None:
            print(f"{job['name']}: no autotest step (job {job['conclusion']})", file=sys.stderr)
            continue
        started = parse_time(step["started_at"])
        completed = parse_time(step["completed_at"])
        if started is None or completed is None:
            print(f"{job['name']}: step did not complete", file=sys.stderr)
            continue
        legs.append(Leg(
            parallel=int(matched.group(1)),
            repeat=int(matched.group(2)),
            seconds=(completed - started).total_seconds(),
            conclusion=step["conclusion"],
            url=job["html_url"],
        ))
    return sorted(legs, key=lambda x: (x.parallel, x.repeat))


def print_table(legs):
    by_parallel = collections.defaultdict(list)
    for leg in legs:
        by_parallel[leg.parallel].append(leg)

    if not legs:
        return
    baseline = min(by_parallel)
    baseline_mean = sum(x.seconds for x in by_parallel[baseline]) / len(by_parallel[baseline])

    print(f"{'parallel':>8} {'runs':>5} {'mean':>9} {'min':>9} {'max':>9} "
          f"{'speedup':>8}  results")
    for parallel in sorted(by_parallel):
        group = by_parallel[parallel]
        times = [x.seconds for x in group]
        mean = sum(times) / len(times)
        results = ",".join(x.conclusion or "?" for x in group)
        print(f"{parallel:>8} {len(group):>5} {mean/60:>8.1f}m {min(times)/60:>8.1f}m "
              f"{max(times)/60:>8.1f}m {baseline_mean/mean:>7.2f}x  {results}")


def plot(legs, path):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not available; skipping graph", file=sys.stderr)
        return

    by_parallel = collections.defaultdict(list)
    for leg in legs:
        by_parallel[leg.parallel].append(leg)
    xs = sorted(by_parallel)
    means = [sum(x.seconds for x in by_parallel[p]) / len(by_parallel[p]) / 60 for p in xs]

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot(xs, means, "-o", color="tab:blue", label="mean of runs")
    for parallel in xs:
        for leg in by_parallel[parallel]:
            colour = "tab:green" if leg.conclusion == "success" else "tab:red"
            ax.plot(parallel, leg.seconds / 60, "x", color=colour)

    ax.set_xlabel("--parallel")
    ax.set_ylabel("wall-clock time of the Rover suite (minutes)")
    ax.set_title("Rover autotest suite vs --parallel on a GitHub runner")
    ax.grid(True, alpha=0.3)
    ax.set_xticks(xs)
    ax.legend(["mean of runs", "individual run (green pass / red fail)"])
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    print(f"wrote {path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", default="peterbarker/ardupilot")
    parser.add_argument("--branch", default="pr-claude2/ci/rover-parallel-sweep")
    parser.add_argument("--workflow", default="test_sitl_rover.yml")
    parser.add_argument("--run-id", type=int, default=None,
                        help="run to report on; default is the latest on --branch")
    parser.add_argument("--plot", default=None, help="write a PNG graph here")
    args = parser.parse_args()

    run_id = args.run_id
    if run_id is None:
        run_id = latest_run_id(args.repo, args.branch, args.workflow)
        print(f"reporting on run {run_id}")

    legs = legs_from_jobs(fetch_jobs(args.repo, run_id))
    print_table(legs)
    if args.plot is not None:
        plot(legs, args.plot)


if __name__ == '__main__':
    main()
