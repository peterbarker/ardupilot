#!/usr/bin/env python

'''
Stripped down version of mavlogdump.py.
'''
from __future__ import print_function

import fnmatch
import os
import re
import sys
import time

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
parser.add_argument("--debug", action='store_true', help="debug mode")

args = parser.parse_args()

from pymavlink import mavutil

def debug(msg):
    if not args.debug:
        return
    print(msg)

filename = args.log

# calculate a scaling factor to move ADC2 to match ADC1
sum_adc1 = 0
sum_adc2 = 0
point_count = 0
tstart = time.time()

buckets_adc = {}
buckets_adc1 = {}
buckets_adc2 = {}

if True:
    mlog = mavutil.mavlink_connection(filename,
                                      robust_parsing=args.robust,
                                      zero_time_base=args.zero_time_base)
    while True:
        m = mlog.recv_match(type='ADCL')
        if m is None:
            break
        if m.ADC1 == 0 or m.ADC2 == 0:
            continue
        sum_adc1 += m.ADC1
        sum_adc2 += m.ADC2
        if m.ADC1 not in buckets_adc1:
            buckets_adc1[m.ADC1] = 0
        buckets_adc1[m.ADC1] += 1
        if m.ADC2 not in buckets_adc2:
            buckets_adc2[m.ADC2] = 0
        buckets_adc2[m.ADC2] += 1

        if m.ADC1 not in buckets_adc:
            buckets_adc[m.ADC1] = 0
        if m.ADC2 not in buckets_adc:
            buckets_adc[m.ADC2] = 0
        buckets_adc[m.ADC1] += 1
        buckets_adc[m.ADC2] += 1

        point_count += 1

if sum_adc2 == 0:
   print("No non-zero adc2 points?")
   sys.exit(1)

scale_adc2 = (sum_adc1 / sum_adc2)

debug("Scaling factor: %s (%s/%s) points=%u" %
      (str(scale_adc2), sum_adc1, sum_adc2, point_count))

def dump_bucket(name, bucket):
    for key in sorted(bucket.keys()):
        print("%f %u" % (key, bucket[key]))

dump_bucket("ADC", buckets_adc)

#dump_bucket("ADC1", buckets_adc1)
#dump_bucket("ADC2", buckets_adc2)
sys.exit(0)

if scale_adc2 < 1:
    raise ValueError("Expected to be scaling up adc2")
sys.exit(0)

mlog = mavutil.mavlink_connection(filename, 
                                  robust_parsing=args.robust,
                                  zero_time_base=args.zero_time_base)

class Column:
    def __init__(self, msg, field, function=None, units=None, print_fmt=None, heading=None):
        self.msg = msg
        self.field = field
        self.units = units
        self.print_fmt = print_fmt
        self.heading = heading
        self.function = function

columns = [
    Column(None, None, heading='Date', function=lambda:time.strftime('%Y-%m-%d', time.gmtime(m._timestamp))),
    Column(None, None, heading='Time', function=lambda:time.strftime('%H:%M:%S', time.gmtime(m._timestamp))),
    Column('GPS','Lat', units='DegreesLatitude', print_fmt="{:.8f}"),
    Column('GPS','Lng', units='DegreesLongitude', print_fmt="{:.8f}"),
    Column('GPS','Spd', units='metres_per_second', print_fmt="{:.8f}"),
    Column('GPA','HAcc', units='metres'),
    Column('GPS','Status'),
    Column('NKF1', 'VN', units='metres_per_second'),
    Column('NKF1', 'VE', units='metres_per_second'),
    Column('POS', 'Lat', units='DegreesLatitude'),
    Column('POS', 'Lng', units='DegreesLongitude'),
    Column('ADCL', 'ADC1'),
    Column('ADCL', 'ADC2'),
    Column(None, None, function=lambda:((last_msgs["ADCL"].ADC1+last_msgs["ADCL"].ADC2)*1000), heading="Lux"),
    Column('ATT', 'Pitch', units='degrees'),
    Column('ATT', 'Roll', units='degrees'),
    Column('IMU','AccX', units='metres_per_second_per_second'),
    Column('IMU','AccY', units='metres_per_second_per_second'),
    Column('IMU','AccZ', units='metres_per_second_per_second'),
]

types = {}
for column in columns:
    if column.msg is None:
        continue
    types[column.msg] = 1

ext = os.path.splitext(filename)[1]
isbin = ext in ['.bin', '.BIN']

if not (isbin):
    print("Need bin")
    quit()

if args.csv_sep == "tab":
    args.csv_sep = "\t"

def match_type(mtype, patterns):
    '''return True if mtype matches pattern'''
    for p in patterns:
        if fnmatch.fnmatch(mtype, p):
            return True
    return False

last_msgs = {}

csv_out = [
    "timestamp",
]
for column in columns:
    if column.heading is not None:
        heading = column.heading
    else:
        heading = "_".join([column.msg,column.field])
    if column.units is not None:
        heading += "_in_" + column.units
    if re.search("[\s;,]", heading):
        raise Exception("Invalid heading (%s)" % str(heading))
    csv_out.append(heading)

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
        if column.msg is None:
            value = column.function()
        else:
            value = last_msgs[column.msg].__getattr__(column.field) # getattr itself is overridden!
        if column.print_fmt is None:
            value = str(value)
        else:
            value = column.print_fmt.format(value)
        csv_out.append(value)

    print(args.csv_sep.join(csv_out))
