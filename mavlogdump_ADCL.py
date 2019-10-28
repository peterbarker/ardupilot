#!/usr/bin/env python

'''
example program that dumps a Mavlink log file. The log file is
assumed to be in the format that qgroundcontrol uses, which consists
of a series of MAVLink packets, each with a 64 bit timestamp
header. The timestamp is in microseconds since 1970 (unix epoch)
'''
from __future__ import print_function

import array
import fnmatch
import json
import os
import struct
import sys
import time

try:
    from pymavlink.mavextra import *
except:
    print("WARNING: Numpy missing, mathematical notation will not be supported..")

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)

parser.add_argument("--robust", action='store_true', help="Enable robust parsing (skip over bad data)")
parser.add_argument("-o", "--output", default=None, help="output matching packets to give file")
parser.add_argument("--csv_sep", dest="csv_sep", default=",", help="Select the delimiter between columns for the output CSV file. Use 'tab' to specify tabs.")
parser.add_argument("--zero-time-base", action='store_true', help="use Z time base for DF logs")
parser.add_argument("--no-bad-data", action='store_true', help="Don't output corrupted messages")
parser.add_argument("log", metavar="LOG")

args = parser.parse_args()

import inspect

from pymavlink import mavutil

filename = args.log
mlog = mavutil.mavlink_connection(filename, 
                                  robust_parsing=args.robust,
                                  zero_time_base=args.zero_time_base)

output = None
if args.output:
    output = open(args.output, mode='wb')

types = [ 'ADCL', 'GPS' ]

ext = os.path.splitext(filename)[1]
isbin = ext in ['.bin', '.BIN', '.px4log']
islog = ext in ['.log', '.LOG'] # NOTE: "islog" does not mean a tlog

if not (isbin or islog):
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
fields = ['timestamp', 'lat', 'lng']
offsets = {}

# Track the last timestamp value. Used for compressing data for the CSV output format.
last_timestamp = None

# Track last GPS lat/lng
last_lat = 0.0
last_lng = 0.0

# for DF logs pre-calculate types list
match_types=['FMT', 'FMTU', 'UNIT', 'MULT', 'GPS', 'ADCL']

# Keep track of data from the current timestep. If the following timestep has the same data, it's stored in here as well. Output should therefore have entirely unique timesteps.
while True:
    m = mlog.recv_match(type=match_types)
    if m is None:
        # write the final csv line before exiting
        if csv_out:
            csv_out[0] = "{:.8f}".format(last_timestamp)
            csv_out[1] = "{:.8f}".format(last_lat)
            csv_out[2] = "{:.8f}".format(last_lng)
            print(args.csv_sep.join(csv_out))
        break

    if (isbin or islog) and m.get_type() == "FMT":
        if m.Name == types[0]:
            fields += m.Columns.split(',')
            try:
                fields.remove('TimeUS')
            except ValueError:
                pass
            csv_out = ["" for x in fields]
            print(args.csv_sep.join(fields))

    if output is not None:
        if (isbin or islog) and m.get_type() in ["FMT", "FMTU", "UNIT", "MULT"]:
            output.write(m.get_msgbuf())
            continue

    if m.get_type() != 'BAD_DATA' and not match_type(m.get_type(), types):
        continue

    # Ignore BAD_DATA messages is the user requested or if they're because of a bad prefix. The
    # latter case is normally because of a mismatched MAVLink version.
    if m.get_type() == 'BAD_DATA' and (args.no_bad_data is True or m.reason == "Bad prefix"):
        continue

    if m.get_type() == 'GPS':
        last_lat = m.Lat
        last_lng = m.Lng
        continue

    # Grab the timestamp.
    timestamp = getattr(m, '_timestamp', 0.0)

    # If we're just logging, pack in the timestamp and data into the output file.
    if output:
        try:
            output.write(m.get_msgbuf())
        except Exception as ex:
            print("Failed to write msg %s: %s" % (m.get_type(), str(ex)))

    # CSV format outputs columnar data with a user-specified delimiter
    data = m.to_dict()
    type = m.get_type()

    # If this message has a duplicate timestamp, copy its data into the existing data list. Also
    # do this if it's the first message encountered.
    if timestamp == last_timestamp or last_timestamp is None:
        newData = [str(data[y]) if y not in ["timestamp", "lat", "lng"] else "" for y in fields]

        for i, val in enumerate(newData):
            if val:
                csv_out[i] = val

    # Otherwise if this is a new timestamp, print out the old output data, and store the current message for later output.
    else:
        csv_out[0] = "{:.8f}".format(last_timestamp)
        csv_out[1] = "{:.8f}".format(last_lat)
        csv_out[2] = "{:.8f}".format(last_lng)
        print(args.csv_sep.join(csv_out))

        csv_out = [str(data[y]) if y not in ["timestamp", "lat", "lng"] else "" for y in fields]

    # Update our last timestamp value.
    last_timestamp = timestamp
