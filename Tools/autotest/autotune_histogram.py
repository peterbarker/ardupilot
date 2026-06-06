#!/usr/bin/env python3
"""Histogram the autotune CSV(s) produced by autotune_repro.py.

Greedy-clusters each gain into buckets (values within the test's 2% tolerance
are the same bucket) and prints counts, flagging buckets the current AUTOTUNE
test in Tools/autotest/arduplane.py does NOT accept.  Throwaway investigation
tooling -- do not merge.
"""
import csv
import sys

# buckets the current AUTOTUNE test accepts (param -> list of centres) and the
# per-parameter tolerance the test actually uses (see arduplane.py AUTOTUNE).
KNOWN = {
    "PTCH_RATE_P": [1.746079683, 1.343138218, 2.26990366],
    "PTCH_RATE_D": [0.108, 0.141, 0.049, 0.0836, 0.0380],
    "RLL_RATE_P":  [1.222702146],
    "RLL_RATE_D":  [0.070284024, 0.091369226],
    "RLL_RATE_FF": [0.229291457],
    "PTCH_RATE_FF": [0.503520715],
}
KNOWN_TOL = {  # fraction; PTCH_RATE_FF is checked at 5%, everything else at 2%
    "PTCH_RATE_FF": 0.05,
}
TOL = 0.02  # clustering tolerance (2%)


def matches_known(param, v):
    tol = KNOWN_TOL.get(param, 0.02)
    for c in KNOWN.get(param, []):
        if c == 0:
            if v == 0:
                return c
        elif abs(v - c) <= abs(c) * tol:
            return c
    return None


def cluster(values, tol=TOL):
    clusters = []
    for v in sorted(values):
        for cl in clusters:
            mean = sum(cl) / len(cl)
            if abs(v - mean) <= abs(mean) * tol:
                cl.append(v)
                break
        else:
            clusters.append([v])
    return clusters


def main(paths):
    rows = []
    for path in paths:
        with open(path) as f:
            rows.extend(csv.DictReader(f))
    print(f"total rows: {len(rows)}")
    ok = [r for r in rows if r.get("ok") == "1"]
    print(f"ok (autotune converged): {len(ok)}   failed: {len(rows) - len(ok)}")
    print()
    for param in ["PTCH_RATE_P", "PTCH_RATE_D", "RLL_RATE_P", "RLL_RATE_D",
                  "RLL_RATE_FF", "PTCH_RATE_FF"]:
        vals = []
        for r in ok:
            try:
                vals.append(float(r[param]))
            except (ValueError, KeyError, TypeError):
                pass
        if not vals:
            continue
        print(f"== {param} ==  (n={len(vals)})")
        clusters = sorted(cluster(vals), key=lambda cl: -len(cl))
        for cl in clusters:
            mean = sum(cl) / len(cl)
            pct = 100.0 * len(cl) / len(vals)
            tag = "covered" if matches_known(param, mean) is not None else "*** UNCOVERED ***"
            print(f"   {mean:10.5f}  n={len(cl):3d}  {pct:5.1f}%   "
                  f"[{min(cl):.5f}..{max(cl):.5f}]  {tag}")
        print()


if __name__ == "__main__":
    args = sys.argv[1:] or ["autotune-all.csv"]
    main(args)
