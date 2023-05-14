#!/bin/bash

set -e
set -x

cat tune.parm config.parm DJI.parm esp-012.inc >../Chimera-DJI/defaults.parm
cat tune.parm config.parm DJI.parm rfd900x.inc >../Chimera-DJI-RFD900x/defaults.parm

cat tune.parm config.parm Crossfire.parm esp-012.inc >../Chimera-XFire-Ana/defaults.parm

cat tune.parm config.parm SharkByte.parm Spektrum.parm esp-012.inc >../Chimera-Spektrum-SharkByte/defaults.parm
