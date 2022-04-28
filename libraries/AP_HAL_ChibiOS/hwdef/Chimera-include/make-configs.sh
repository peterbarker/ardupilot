#!/bin/bash

set -e
set -x

cat tune.parm config.parm DJI.parm >../Chimera-DJI/defaults.parm
cat tune.parm config.parm Crossfire.parm >../Chimera-XFire-Ana/defaults.parm

cat tune.parm config.parm SharkByte.parm Spektrum.parm >../Chimera-Spektrum-SharkByte/defaults.parm
