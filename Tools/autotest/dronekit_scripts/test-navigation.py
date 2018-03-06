#!/usr/bin/env python

from __future__ import print_function

import argparse
import time

import dronekit

# parse arguments
parser = argparse.ArgumentParser(
    description='Check ArduCopter can take off and go to a location'
)
parser.add_argument('--connect', help="Vehicle connection target string.")
args = parser.parse_args()

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % args.connect)
vehicle = dronekit.connect(args.connect, wait_ready=True)

print("Waiting for vehicle to be armable")
vehicle.wait_for_armable(timeout=lambda : print("Waiting for armability"))

print("Setting mode to GUIDED")
vehicle.wait_for_mode("GUIDED")

print("Arming")
vehicle.arm()

print("Taking off!")
vehicle.wait_simple_takeoff(10)

location = vehicle.location.global_relative_frame

target_location = location
target_location.lat += 0.000001

print("Goto: %f %f %d" % target_location.lat, target_location.lon, target_location.alt)
vehicle.simple_goto(target_location, groundspeed=10)

while True:
    loc = vehicle.location.global_relative_frame
    delta = target_location - location
    print("loc: %s" % str(loc))
    print("delta: %s" % str(delta))

vehicle.close()
