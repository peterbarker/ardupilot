#!/usr/bin/env bash

set -e
set -x

# it is expected that you've done a "./waf configure --board=sitl --debug"
# before running this script.

IMU_DATA_FILE="./Tools/Replay/sample-input/2020_04_09_07_48_33_ADIS16465.csv"
POS_DATA_FILE="./Tools/Replay/sample-input/2020_04_09_07_48_33_UBLOX.pos"

POS_DATA_FILE="./Tools/Replay/sample-input/2020_04_09_07_48_33_UBLOX_interp_5Hz.pos"

reset
#./waf configure --board=sitl
./waf build --target=tools/Replay
gdb --args ./build/sitl/tools/Replay $IMU_DATA_FILE $POS_DATA_FILE
#./build/sitl/tools/Replay $IMU_DATA_FILE $POS_DATA_FILE
