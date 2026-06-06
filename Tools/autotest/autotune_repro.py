#!/usr/bin/env python3
"""Run ONE ArduPlane autotune and append the resulting tune gains to a CSV.

This is throwaway tooling for investigating the intermittent
``Tools/autotest/arduplane.py`` AUTOTUNE failure, where autotune settles on a
discrete PTCH_RATE_P/PTCH_RATE_D "bucket" that the test does not enumerate
(e.g. PTCH_RATE_P=1.589).  Unlike the real AUTOTUNE test it does *not* assert
the gains -- it just records whatever autotune produced, so we can build a
histogram of the solution buckets across many runs.

Designed to be invoked once per fresh process (loop it from the workflow) so
every run starts pristine, exactly like CI runs the AUTOTUNE test.  It drives
the real TestSuite.autotest() -> run_tests() path so RC defaults, mode-switch
poll and mission clearing happen exactly as in CI.

Env:
  AT_CSV     output CSV path (appended; header written if new)
  AT_RUN_ID  integer label for this run (default 0)
  AT_SPEEDUP SITL speedup (default 100, matching the real Plane autotest)
"""
import os
import sys
import csv
import time

AUTOTEST = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, AUTOTEST)

import vehicle_test_suite               # noqa: E402
import arduplane                        # noqa: E402

CSV_PATH = os.environ.get("AT_CSV", "autotune_results.csv")
RUN_ID = int(os.environ.get("AT_RUN_ID", "0"))
SPEEDUP = int(os.environ.get("AT_SPEEDUP", "100"))
RECORD = ["PTCH_RATE_P", "PTCH_RATE_D", "PTCH_RATE_FF",
          "RLL_RATE_P", "RLL_RATE_D", "RLL_RATE_FF"]

binary = os.path.join(os.getcwd(), "build", "sitl", "bin", "arduplane")

tester = arduplane.AutoTestPlane(
    binary,
    speedup=SPEEDUP,
    logs_dir="/tmp/autotune_repro_logs",
    _show_test_timings=False,
)
os.makedirs("/tmp/autotune_repro_logs", exist_ok=True)

if not os.path.exists(CSV_PATH) or os.path.getsize(CSV_PATH) == 0:
    with open(CSV_PATH, "w", newline="") as f:
        csv.writer(f).writerow(["run", "ok", "secs"] + RECORD)

_t0 = time.time()


def AutotuneRecord():
    '''Record autotune result.'''
    ok = True
    vals = {p: float("nan") for p in RECORD}
    try:
        tester.run_autotune()
    except Exception:  # noqa: BLE001
        ok = False
        raise
    finally:
        try:
            vals = {p: tester.get_parameter(p) for p in RECORD}
        except Exception:  # noqa: BLE001
            pass
        secs = round(time.time() - _t0, 1)
        with open(CSV_PATH, "a", newline="") as f:
            csv.writer(f).writerow(
                [RUN_ID, int(ok), secs] + [vals.get(p, float("nan")) for p in RECORD])
        tester.progress("RUN %u RESULT ok=%s secs=%s %s" % (RUN_ID, ok, secs, vals))


passed = tester.autotest(tests=[vehicle_test_suite.Test(AutotuneRecord)],
                         allow_skips=False)
print("RUN %u DONE passed=%s -> %s" % (RUN_ID, passed, CSV_PATH))
# Always exit 0: a "failed" run (autotune raised) is still a recorded data
# point, and we never want one bad autotune to abort the batch.
sys.exit(0)
