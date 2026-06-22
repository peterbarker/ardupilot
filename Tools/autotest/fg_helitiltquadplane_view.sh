#!/bin/sh
# FlightGear view for the helitiltquadplane tilting variable-pitch quadplane
# (plane-heliquad / plane-heliquad-hvec SITL frames).
# Start SITL with the flightgear view output enabled, then run this.
# HELITILT_AIRCRAFT=helitiltquadplane-nice selects the
# presentation-quality model; the default is the schematic one.

AUTOTESTDIR=$(dirname $0)

# FlightGear evaluates --timeofday=noon at the starting position, and the
# external FDM then teleports the model to wherever SITL's home is -- so
# the start position must match SITL home or "noon" can land at night.
# Default is ArduPilot's default home (CMAC); override for other homes,
# e.g. FG_START_POS="--airport=KSFO".  Note the packaged FlightGear base
# data only ships San Francisco area scenery; elsewhere is ocean unless
# scenery has been fetched (e.g. one run with terrasync enabled).
FG_START_POS=${FG_START_POS:-"--lat=-35.363261 --lon=149.165230"}

# --in-air skips FGATCManager registering the (externally driven) user
# aircraft with ground/tower ATC, which otherwise spams "AI error:
# requesting ATC instruction for aircraft without traffic record" once
# SITL moves the aircraft away from where FlightGear placed it.
# The built-in http server exposes the property tree while the view
# runs: browse http://localhost:5505/ (Phi interface) or query JSON,
# e.g.  curl 'http://localhost:5505/json/engines/engine[2]?d=2'
# shows rotor 3's rpm as the animations see it.

# Start in the external Helicopter View, pulled back to the
# chase-distance set in the aircraft -set.xml.  This cannot be done
# from the -set.xml or the command line: views are not built yet when
# those are read, so nudge the property once the sim is up.
(
    until curl -s -m 2 -o /dev/null "http://localhost:5505/json/sim"; do
        sleep 2
    done
    curl -s -o /dev/null -X POST -H 'Content-Type: application/json' \
        -d '{"value": 1}' \
        "http://localhost:5505/json/sim/current-view/view-number"
) &

nice fgfs \
    --httpd=5505 \
    --native-fdm=socket,in,10,,5503,udp \
    --fdm=external \
    --aircraft=${HELITILT_AIRCRAFT:-helitiltquadplane} \
    --fg-aircraft="$AUTOTESTDIR/aircraft" \
    $FG_START_POS \
    --in-air \
    --geometry=650x550 \
    --bpp=32 \
    --disable-hud-3d \
    --disable-horizon-effect \
    --timeofday=noon \
    --disable-sound \
    --disable-fullscreen \
    --disable-random-objects \
    --disable-ai-models \
    --disable-terrasync \
    --prop:/sim/traffic-manager/enabled=false \
    --prop:/sim/ai/scenarios-enabled=false \
    --fog-disable \
    --disable-specular-highlight \
    --disable-anti-alias-hud \
    --wind=0@0 \
    $*
