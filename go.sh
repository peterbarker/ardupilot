#!/bin/bash

#./Tools/scripts/sitl-on-hardware/sitl-on-hw.py \
#    --board CubeOrange \
#    -v Plane
#exit

# cat >/tmp/extra.hwdef <<"EOF"
# define AP_SIM_FLIGHTAXIS_ENABLED 1
# EOF

# ./Tools/scripts/sitl-on-hardware/sitl-on-hw.py \
#     --board CubeRedPrimary \
#     -v Plane \
#     --extra-hwdef /tmp/extra.hwdef \
#     --frame "flightaxis:172.16.3.142" \
#     --simclass FlightAxis


cat >/tmp/extra.hwdef <<"EOF"
define AP_SIM_FLIGHTAXIS_ENABLED 1
define AP_NETWORKING_ENABLED 1
define AP_NETWORKING_BACKEND_PPP 1
define AP_NETWORKING_CONTROLS_HOST_IP_SETTINGS_ENABLED 1
EOF

./Tools/scripts/sitl-on-hardware/sitl-on-hw.py \
    --board CubeOrange \
    -v Plane \
    --extra-hwdef /tmp/extra.hwdef \
    --frame "flightaxis:172.16.3.142" \
    --simclass FlightAxis \
    --debug \
    --disable-watchdog


cat >/dev/null <<"EOF"
param set NET_ENABLE 1
param set NET_OPTIONS 1
param set SERIAL1_PROTOCOL 48
param set SERIAL1_BAUD 12500000


# set up on the network
param set NET_P1_IP0       172.0
param set NET_P1_IP1       16.0
param set NET_P1_IP2       3.0
param set NET_P1_IP3       162.0


EOF
