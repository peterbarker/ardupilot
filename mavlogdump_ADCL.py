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

args = parser.parse_args()


def debug(msg):
    if not args.debug:
        return
    print(msg, file=sys.stderr)


adc1_x_offset = 2.5 # metres towards front of vehicle
adc2_x_offset = -2.5 # metres towards front of vehicle


# from MAVProxy's mp_util:
def gps_distance(lat1, lon1, lat2, lon2):
    '''return distance between two points in meters,
    coordinates are in degrees
    thanks to http://www.movable-type.co.uk/scripts/latlong.html'''
    lat1 = math.radians(lat1)
    lat2 = math.radians(lat2)
    lon1 = math.radians(lon1)
    lon2 = math.radians(lon2)
    dLat = lat2 - lat1
    dLon = lon2 - lon1

    a = math.sin(0.5*dLat)**2 + math.sin(0.5*dLon)**2 * math.cos(lat1) * math.cos(lat2)
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0-a))
    return mavextra.radius_of_earth * c


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

mlog_adc1 = mavutil.mavlink_connection(
    filename,
    robust_parsing=args.robust,
    zero_time_base=args.zero_time_base
)

types = set(['ADCL', 'GPS', 'GPA', 'NKF1', 'POS', 'ATT', 'IMU'])

print("Looking for all messages present on mlog", file=sys.stderr)
ensure_messages(mlog_adc1, types)


class NeedMatch(object):
    def __init__(self, adcl, location, gps, gpa, nkf1, att):
        self.adcl = adcl
        self.location = location
        self.distance = 0
        self.gps = gps
        self.gpa = gpa
        self.nkf1 = nkf1
        self.att = att


# algorithm:
#  - when we get an ADCL:
#      - its location is set at the current location projected 2.5m forward of the current canonical vehicle position (GPS+accels+math)  # noqa
#      - sum up the distance travelled (speed*time_since_last_ADCL) into each of the things in the list of outstanding items
#      - when the length of one of those vectors reaches 5m:
#           - choose the current ADCL or the previous, whichever gets us closer to 5m
#           - emit a match, location is expiring ADCL measurement, Lux is sum of (current_or_previous_adcl and the expiring ADCL measurement)  # noqa
#       - if the distance travelled is less than 5m and the time delta is greater than 20 seconds then discard this measurement; the vehicle has probably stopped for a long time in this case and summed-movement is probably GPS-drift-dominated  # noqa

needmatch = []

#  headings = ['timestamp', 'Lat', 'Lng', 'ADC1', 'matched_ADC2', 'Lux', 'Speed']
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


def emit_headings_kml(entry):
    lat = entry.location[0]
    lon = entry.location[1]
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

def emit_row_kml(entry, adc_to_use, lux, speed):
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
</Placemark>""".format(name="name-%u" % entry.gps.TimeUS,
           lat=entry.location[0],
           lon=entry.location[1],
           alt=lux * 10000,
           lux=lux,
           speed=speed,
           heading=entry.gps.GCrs,
           time=time.strftime('%Y-%m-%d %H:%M:%S', time.gmtime(entry.gps._timestamp)),
           timestamp=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime(entry.gps._timestamp)))
    pm = re.sub(r"^ +", "", pm, flags=re.M)
    print(pm)


def emit_headings(entry):
    global args
    if args.kml:
        emit_headings_kml(entry)
        return
    print(headings.join(','))


def emit_footer():
    global args
    if args.kml:
        emit_footer_kml()


def emit_row(entry, adc_to_use, lux, speed):
    global args
    if args.kml:
        emit_row_kml(entry, adc_to_use, lux, speed)
        return
    out = [
        "%u" % entry.adcl.TimeUS,
        time.strftime('%Y-%m-%d', time.gmtime(entry.gps._timestamp)),
        time.strftime('%H:%M:%S', time.gmtime(entry.gps._timestamp)),
        "%f" % entry.location[0],
        "%f" % entry.location[1],
        "%f" % lux,
        "%f" % entry.gps.Spd,
        "%f" % entry.gpa.HAcc,
        "%u" % entry.gps.Status,
        "%f" % entry.nkf1.VN,
        "%f" % entry.nkf1.VE,
        "%f" % entry.att.Pitch,
        "%f" % entry.att.Roll,
    ]
    print(','.join(out))

    # entry.adcl.TimeUS,
    # entry.location[0],
    # entry.location[1],
    # entry.adcl.ADC1,
    # adc_to_use.ADC2,
    # lux,
    # speed,
    # ])


print("Starting loop", file=sys.stderr)

count = 0
debug_count = 0
last_adc1 = None
discard_count = 0
match_count = 0
non_zero_match_count = 0
max_rows = None
emitted_rows = 0
emitted_headings = False
while True:
    if max_rows is not None and emitted_rows >= max_rows:
        break
    adc1 = mlog_adc1.recv_match(type=types)
    if adc1 is None:
        break
    if int(mlog_adc1.percent*100) != int(last_percent*100):
        print("\rMatching: %5.2f%% discard=%u match=%u non_noise_matches=%u" %
              (mlog_adc1.percent, discard_count, match_count, non_zero_match_count),
              end='',
              file=sys.stderr)
        last_percent = mlog_adc1.percent
    if only_do_percent is not None and mlog_adc1.percent > only_do_percent:
        print("Only doing first %f percent" % only_do_percent, file=sys.stderr)
        break

    if adc1.get_type() != "ADCL":
        continue

    # skip anything where we're travelling too slowly
    if False:
        NKF1 = mlog_adc1.messages["NKF1"]
        vel = math.sqrt(NKF1.VN*NKF1.VN + NKF1.VE*NKF1.VE)
        if vel < 1.0:
            continue
        if vel > 5:
            debug_count = 0

#    print("adc1: %s yaw=%s" % (str(adc1), mlog_adc1.messages["ATT"].Yaw))

#    if debug:
#        print("map circle %f %f 0.5 red" % (mlog_adc1.messages["POS"].Lat,mlog_adc1.messages["POS"].Lng))
#    print("here: %s" % ((mlog_adc1.messages["POS"].Lat,mlog_adc1.messages["POS"].Lng),))

#    yaw = math.degrees(math.atan(NKF1.VE/NKF1.VN))
    yaw = mlog_adc1.messages["GPS"].GCrs
#    yaw = mlog_adc1.messages["ATT"].Yaw,

    speed = mlog_adc1.messages["GPS"].Spd
#    speed = math.sqrt(mlog_adc1.messages["NKF1"].VN*mlog_adc1.messages["NKF1"].VN + mlog_adc1.messages["NKF1"].VE*mlog_adc1.messages["NKF1"].VE)  # noqa

    projected_forwards_location = mavextra.gps_newpos(
        mlog_adc1.messages["POS"].Lat,
        mlog_adc1.messages["POS"].Lng,
        yaw,
        2.5)

#    dist = gps_distance(projected_adc1_location[0],
#                        projected_adc1_location[1],
#                        projected_backwards_location[0],
#                        projected_backwards_location[1])
#    print("Dist: %s" % str(dist))

#    sys.exit(0)

    need_distance = 5

    max_match_time_delta_us = 20000000

    if last_adc1 is not None:
        time_travelled = adc1.TimeUS - last_adc1.TimeUS
        distance_travelled = (time_travelled * speed) / 1000000.0

        new_needmatch = []
        for entry in needmatch:
            if entry.distance + distance_travelled > need_distance:
                adc_to_use = adc1
                distance_delta = entry.distance + distance_travelled
                if (need_distance - entry.distance) > (entry.distance + distance_travelled) - need_distance:
                    # use previous
                    adc_to_use = last_adc1
                    distance_delta = entry.distance
#                print("Matched (%s) with (%s) after %fm tdelta=%f spd=%f len=%u" % (str(entry.adcl), str(adc_to_use), distance_delta, (adc_to_use.TimeUS-entry.adcl.TimeUS)/1000000.0, speed, len(needmatch)))  # noqa

                # timestamp,Lat,Lng,ADC1,matched_ADC2,Lux,speed
                match_count += 1
                lux = (entry.adcl.ADC1+adc_to_use.ADC2)
                if lux > 0.001:  # Firstly anything below 0.001 should be considered zero.
                    non_zero_match_count += 1
                else:
                    lux = 0
                if not emitted_headings:
                    emit_headings(entry)
                    emitted_headings = True
                emit_row(entry, adc_to_use, lux, speed)
                emitted_rows += 1
            else:
                if adc1.TimeUS - entry.adcl.TimeUS < max_match_time_delta_us:
                    entry.distance += distance_travelled
#                print("Entry now at %fm" % entry.distance)
                    new_needmatch.append(entry)
                else:
                    discard_count += 1
#                    print("Discarding (%s) spd=%f" % (str(entry.adcl), speed,))

        needmatch = new_needmatch

    new_needmatch_entry = NeedMatch(
        adc1,
        projected_forwards_location,
        mlog_adc1.messages["GPS"],
        mlog_adc1.messages["GPA"],
        mlog_adc1.messages["NKF1"],
        mlog_adc1.messages["ATT"],
    )
    needmatch.append(new_needmatch_entry)

    last_adc1 = adc1

emit_footer()
