#!/bin/bash

python3.9 ./Tools/autotest/sim_vehicle.py \
    --vehicle plane \
    --frame quadplane \
    --aircraft Volanti \
    --gdb \
    --debug \
    --map \
    --console \
    -l -35.58324108,148.97773215,1119.1,45.0 \
    $*

# fire: -35.5882733 148.9698032

# high points
# -35.59614497 148.97168085
# -35.58830957 148.98004853


# export PATH=/home/pbarker/bin:/home/pbarker/.local/bin:/home/pbarker/rc/sdcc/sdcc/bin:/usr/lib/ccache:/home/pbarker/gcc/bin:/usr/lib/ccache:/home/pbarker/rc/jsbsim/build/src:/home/pbarker/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin:/snap/bin:/home/pbarker/rc/ardupilot/Tools/autotest:/home/pbarker/rc/stlink/build/Release
