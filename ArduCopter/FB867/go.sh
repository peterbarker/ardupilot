#!/usr/bin/env bash

../../Tools/autotest/sim_vehicle.py \
    --gdb --debug \
    --vehicle ArduCopter \
    -l -35.269930,149.108749,620.4,0 \
    --mavproxy-script=$PWD/mavproxy.scr \
    --map \
    --console

