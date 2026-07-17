#!/usr/bin/env bash
# Temporary CI soak harness - DO NOT MERGE
#
# CONTROL RUN: unmodified master CANGPSCopterMission, to determine
# whether its failure in the small-wallclock-fixes soak is caused by
# that branch's change or is pre-existing under CI runner conditions.

set -ex

waf=modules/waf/waf-light

SOAK_BUILD="build.SITLPeriphUniversal"
SOAK_TESTS="test.CAN.CANGPSCopterMission"
SOAK_ITERS=8

export PIP_ROOT_USER_ACTION=ignore
export SITL_PANIC_EXIT=1
export GIT_VERSION="abcdef"
export GIT_VERSION_EXTENDED="0123456789abcdef"
export GIT_VERSION_INT="15"

echo "::group::install test dependencies"
git submodule update --init --recursive --depth 1
pushd /tmp
  git clone https://github.com/ardupilot/MAVProxy --depth 1
  pushd MAVProxy
    python3 -m pip install --progress-bar off --cache-dir /tmp/pip-cache --user --force .
  popd
popd
# now uninstall the version of pymavlink pulled in by MAVProxy deps and
# use the repository version:
python3 -m pip uninstall -y pymavlink --cache-dir /tmp/pip-cache
(cd modules/mavlink/pymavlink && python3 -m pip install --progress-bar off --cache-dir /tmp/pip-cache --user .)
echo "::endgroup::"

cat /proc/cpuinfo

echo "Building SITL Periph GPS"
$waf configure --board sitl
$waf copter

STRESS="${CI_SOAK_STRESS:-0}"
STRESS_PIDS=()
if [ "$STRESS" -gt 0 ]; then
    echo "Starting $STRESS CPU stressers"
    for i in $(seq "$STRESS"); do
        yes > /dev/null &
        STRESS_PIDS+=($!)
    done
fi
trap 'kill "${STRESS_PIDS[@]}" 2>/dev/null || true' EXIT

Tools/autotest/autotest.py --show-test-timings $SOAK_BUILD

for i in $(seq "$SOAK_ITERS"); do
    echo "=== soak iteration $i/$SOAK_ITERS (stress=$STRESS) ==="
    Tools/autotest/autotest.py --show-test-timings $SOAK_TESTS
done
