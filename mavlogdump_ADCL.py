#!/usr/bin/env python

'''
Stripped down version of mavlogdump.py.
'''
from __future__ import print_function

import fnmatch
import os

try:
    from pymavlink.mavextra import *
except:
    print("WARNING: Numpy missing, mathematical notation will not be supported..")

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)

parser.add_argument("--robust", action='store_true', help="Enable robust parsing (skip over bad data)")
parser.add_argument("--csv_sep", dest="csv_sep", default=",", help="Select the delimiter between columns for the output CSV file. Use 'tab' to specify tabs.")
parser.add_argument("--zero-time-base", action='store_true', help="use Z time base for DF logs")
parser.add_argument("log", metavar="LOG")

args = parser.parse_args()

from pymavlink import mavutil

filename = args.log
mlog = mavutil.mavlink_connection(filename, 
                                  robust_parsing=args.robust,
                                  zero_time_base=args.zero_time_base)

columns = [
    ('GPS','Lat', "{:.8f}"),
    ('GPS','Lng', "{:.8f}"),
    ('GPS','Spd', "{:.8f}"),
    ('GPS','HDop', None),
    ('GPS','Status', None),
    ('NKF1', 'VN', None),
    ('NKF1', 'VE', None),
    ('POS', 'Lat', None),
    ('POS', 'Lng', None),
    ('ADCL','ADC1', None),
    ('ADCL','ADC2', None),
    ('ATT', 'Pitch', None),
    ('ATT', 'Roll', None),
    ('IMU','AccX', None),
    ('IMU','AccY', None),
    ('IMU','AccZ', None),
]

types = {}
for column in columns:
    types[column[0]] = 1

ext = os.path.splitext(filename)[1]
isbin = ext in ['.bin', '.BIN']

if not (isbin):
    print("Need bin or log file")
    quit()

if args.csv_sep == "tab":
    args.csv_sep = "\t"

def match_type(mtype, patterns):
    '''return True if mtype matches pattern'''
    for p in patterns:
        if fnmatch.fnmatch(mtype, p):
            return True
    return False

# Write out a header row as we're outputting in CSV format.
fields = ['timestamp', 'lat', 'lng', "spd"]

last_msgs = {}

csv_out = [
    "timestamp",
]
for column in columns:
    csv_out.append(".".join([column[0],column[1]]))

print(args.csv_sep.join(csv_out))

# Keep track of data from the current timestep. If the following timestep has the same data, it's stored in here as well. Output should therefore have entirely unique timesteps.
while True:
    m = mlog.recv_match(type=list(types.keys()))
    if m is None:
        break

    last_msgs[m.get_type()] = m

    # we emit on each ADCL message:
    if m.get_type() != 'ADCL':
        continue

    # we must have one of each message type to continue:
    have_all = True
    for i in types:
        if i not in last_msgs:
            have_all = False
            break
    if not have_all:
        continue

    # CSV format outputs columnar data with a user-specified delimiter
    csv_out = [
        "{:.8f}".format(m._timestamp),
    ]
    for column in columns:
        (msgname, field, fmt) = column
        value = last_msgs[msgname].__getattr__(field) # getattr itself is overridden!
        if fmt is None:
            value = str(value)
        else:
            value = fmt.format(value)
        csv_out.append(value)

    print(args.csv_sep.join(csv_out))
