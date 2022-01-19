#!/usr/bin/env python

'''
Stripped down version of mavlogdump.py.
'''
from __future__ import print_function

import sys
import time
import math
import re

from pymavlink import mavextra
from pymavlink import mavutil

try:
    from pymavlink.mavextra import *  # noqa
except Exception:
    print("WARNING: Numpy missing, mathematical notation will not be supported..")

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)

parser.add_argument("--robust", action='store_true', help="Enable robust parsing (skip over bad data)")
parser.add_argument(
    "--csv_sep",
    dest="csv_sep",
    default=",",
    help="Select the delimiter between columns for the output CSV file. Use 'tab' to specify tabs."
)
parser.add_argument("--zero-time-base", action='store_true', help="use Z time base for DF logs")
parser.add_argument("log", metavar="LOG")
parser.add_argument("--debug", action='store_true', help="debug mode")
parser.add_argument("--kml", action='store_true', help="emit kml")
parser.add_argument("--tz-offset-minutes", type=int, help="minutes to add to each output timestamp", default=0)

args = parser.parse_args()


def debug(msg):
    if not args.debug:
        return
    print(msg, file=sys.stderr)


def ensure_messages(an_mlog, types):
    # step mlog forward until we have all required messages
    while True:
        current_adc2 = an_mlog.recv_match(type=types)
        if current_adc2 is None:
            raise Exception("Required messages not found")
        # we must have one of each message type to continue:
        have_all = True
        for i in types:
            if i not in an_mlog.messages:
                have_all = False
                break
        if have_all:
            break


only_do_percent = 5
only_do_percent = None


last_percent = -1

filename = args.log

mlog = mavutil.mavlink_connection(
    filename,
    robust_parsing=args.robust,
    zero_time_base=args.zero_time_base
)

types = set(['ADCL', 'GPS', 'GPA', 'NKF1', 'POS', 'ATT', 'IMU'])

print("Looking for all messages present on mlog", file=sys.stderr)
ensure_messages(mlog, types)

headings = [
    'timestamp',
    'Date',
    'Time',
    'Lat',
    'Lng',
    'Lux',
    'GPS_Spd',
    'GPA_HAcc',
    'GPS_Status',
    'NKF1.VN',
    'NKF1.VE',
    'ATT.Pitch',
    'ATT.Roll',
]


def emit_headings_kml(mlog):
    gps = mlog.messages["GPS"]
    lat = gps.Lat
    lon = gps.Lng
    print("""<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2" xmlns:gx="http://www.google.com/kml/ext/2.2">
  <Document>
    <name>Lux Graph</name>
    <snippet>Created Fri Nov 24 11:08:37 2017</snippet>
    <LookAt>
      <gx:TimeSpan>
        <begin>2017-11-12T12:26:18Z</begin>
        <end>2017-11-12T13:42:12Z</end>
      </gx:TimeSpan>
      <longitude>{lon}</longitude>
      <latitude>{lat}</latitude>
      <range>1300.000000</range>
    </LookAt>
    <!-- Normal track style -->
    <Style id="track_n">
      <IconStyle>
        <scale>.5</scale>
        <Icon>
          <href>http://earth.google.com/images/kml-icons/track-directional/track-none.png</href>
        </Icon>
      </IconStyle>
      <LabelStyle>
        <scale>0</scale>
      </LabelStyle>
    </Style>
    <!-- Highlighted track style -->
    <Style id="track_h">
      <IconStyle>
        <scale>1.2</scale>
        <Icon>
          <href>http://earth.google.com/images/kml-icons/track-directional/track-none.png</href>
        </Icon>
      </IconStyle>
    </Style>
    <StyleMap id="track">
      <Pair>
        <key>normal</key>
        <styleUrl>#track_n</styleUrl>
      </Pair>
      <Pair>
        <key>highlight</key>
        <styleUrl>#track_h</styleUrl>
      </Pair>
    </StyleMap>
    <!-- Normal waypoint style -->
    <Style id="waypoint_n">
      <IconStyle>
        <Icon>
          <href>http://maps.google.com/mapfiles/kml/pal4/icon61.png</href>
        </Icon>
      </IconStyle>
    </Style>
    <!-- Highlighted waypoint style -->
    <Style id="waypoint_h">
      <IconStyle>
        <scale>1.2</scale>
        <Icon>
          <href>http://maps.google.com/mapfiles/kml/pal4/icon61.png</href>
        </Icon>
      </IconStyle>
    </Style>
    <StyleMap id="waypoint">
      <Pair>
        <key>normal</key>
        <styleUrl>#waypoint_n</styleUrl>
      </Pair>
      <Pair>
        <key>highlight</key>
        <styleUrl>#waypoint_h</styleUrl>
      </Pair>
    </StyleMap>
    <Style id="lineStyle">
      <LineStyle>
        <color>99ffac59</color>
        <width>6</width>
      </LineStyle>
    </Style>
        <Folder>
          <name>Points</name>
    """.format(lat=lat, lon=lon))


def emit_footer_kml():
    print("""
    </Folder>
  </Document>
</kml>
""")


# <LookAt>
#  <longitude>{lon}</longitude>
#  <latitude>{lat}</latitude>
#  <tilt>66</tilt>
# </LookAt>

def emit_row_kml(mlog):
    gps = mlog.messages["GPS"]
    gpa = mlog.messages["GPA"]
    nkf1 = mlog.messages["NKF1"]
    att = mlog.messages["ATT"]
    adcl = mlog.messages["ADCL"]

    lux = adcl.ADC1

    pm = """<Placemark>
 <name>{name}</name>
 <description><![CDATA[
  <table>
  <tr><td>Lon: {lon}</td></tr>
  <tr><td>Lat: {lat}</td></tr>
  <tr><td>Lux: {lux}</td></tr>
  <tr><td>Spd: {speed} m/s</td></tr>
  <tr><td>Hdg: {heading}</td></tr>
  <tr><td>Time: {time}</td></tr>
  </table>]]>
 </description>
 <TimeStamp><when>{timestamp}</when></TimeStamp>
 <styleUrl>#track</styleUrl>
 <Point>
   <altitudeMode>relativeToGround</altitudeMode>
   <extrude>1</extrude>
   <coordinates>{lon},{lat},{alt}</coordinates>
 </Point>
</Placemark>""".format(name="name-%u" % gps.TimeUS,
           lat=gps.Lat,
           lon=gps.Lng,
           alt=lux * 10000,
           lux=lux,
           speed=gps.Spd,
           heading=gps.GCrs,
           time=time.strftime('%Y-%m-%d %H:%M:%S', time.gmtime(gps._timestamp + args.tz_offset_minutes*60)),
           timestamp=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime(gps._timestamp)))
    pm = re.sub(r"^ +", "", pm, flags=re.M)
    print(pm)


def emit_headings(mlog):
    global args
    if args.kml:
        emit_headings_kml(mlog)
        return
    print(','.join(headings))


def emit_footer():
    global args
    if args.kml:
        emit_footer_kml()


def emit_row(mlog):
    global args

    gps = mlog.messages["GPS"]
    gpa = mlog.messages["GPA"]
    nkf1 = mlog.messages["NKF1"]
    att = mlog.messages["ATT"]
    adcl = mlog.messages["ADCL"]

    if args.kml:
        emit_row_kml(mlog)
        return

    lux = adcl.ADC1
    out = [
        "%u" % adcl.TimeUS,
        time.strftime('%Y-%m-%d', time.gmtime(gps._timestamp + args.tz_offset_minutes*60)),
        time.strftime('%H:%M:%S', time.gmtime(gps._timestamp + args.tz_offset_minutes*60)),
        "%f" % gps.Lat,
        "%f" % gps.Lng,
        "%f" % lux,
        "%f" % gps.Spd,
        "%f" % gpa.HAcc,
        "%u" % gps.Status,
        "%f" % nkf1.VN,
        "%f" % nkf1.VE,
        "%f" % att.Pitch,
        "%f" % att.Roll,
    ]
    print(','.join(out))


print("Starting loop", file=sys.stderr)

max_rows = None
emitted_rows = 0
emitted_headings = False
while True:
    if max_rows is not None and emitted_rows >= max_rows:
        break
    adc1 = mlog.recv_match(type=types)
    if adc1 is None:
        break
    if only_do_percent is not None and mlog.percent > only_do_percent:
        print("Only doing first %f percent" % only_do_percent, file=sys.stderr)
        break

    if adc1.get_type() != "ADCL":
        continue

    # skip anything where we're travelling too slowly
    if False:
        NKF1 = mlog.messages["NKF1"]
        vel = math.sqrt(NKF1.VN*NKF1.VN + NKF1.VE*NKF1.VE)
        if vel < 1.0:
            continue

    if not emitted_headings:
        emit_headings(mlog)
        emitted_headings = True
    emit_row(mlog)
    emitted_rows += 1

emit_footer()
