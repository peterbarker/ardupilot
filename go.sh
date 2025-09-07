#!/bin/bash

BOARD=CubeRedPrimary

cat >/tmp/defaults.parm <<"EOF"
NET_ENABLE 1
NET_DHCP 0

NET_IPADDR0      172
NET_IPADDR1      16
NET_IPADDR2      3
NET_IPADDR3      100

# NET_OPTIONS 1
# SERIAL1_PROTOCOL 48
# SERIAL1_BAUD 12500000
EOF

cat >/tmp/extra.hwdef <<"EOF"
define HAL_SIM_JSON_ENABLED 1
# define AP_NETWORKING_ENABLED 1
# define AP_NETWORKING_BACKEND_PPP 1
# define AP_NETWORKING_CONTROLS_HOST_IP_SETTINGS_ENABLED 1

define DISABLE_WATCHDOG 0
EOF

#./Tools/scripts/sitl-on-hardware/sitl-on-hw.py \
#    --board $BOARD \
#    --vehicle plane \
#    --extra-hwdef=/tmp/extra.hwdef \
#    --defaults=/tmp/defaults.parm \
#    --debug

#exit 0

./Tools/scripts/sitl-on-hardware/sitl-on-hw.py \
    --board $BOARD \
    --vehicle plane \
    --simclass JSON \
    --frame json:172.16.3.160 \
    --extra-hwdef=/tmp/extra.hwdef \
    --defaults=/tmp/defaults.parm \
    --debug

