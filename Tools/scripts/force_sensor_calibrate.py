#!/usr/bin/env python3

'''
Calibrate an AP_ForceSensor instance (load cell) over MAVLink.

The vehicle does the work: MAV_CMD_FORCE_SENSOR_CALIBRATE asks it either to
zero a sensor or to work out the sensor's scale factor from a known mass which
is currently applied to it. The vehicle averages a block of samples, saves the
result to FSCLn_ZERO / FSCLn_SCALE and reports the outcome by STATUSTEXT.

Nothing needs to be installed on the vehicle - no scripting, no LUA.

AP_FLAKE8_CLEAN

Examples:

  # what is configured, and what is each sensor's current calibration?
  force_sensor_calibrate.py /dev/ttyACM0 status

  # zero sensor 1 (nothing on the load cell)
  force_sensor_calibrate.py /dev/ttyACM0 tare

  # tell sensor 1 that the 500g currently on it weighs 500g
  force_sensor_calibrate.py /dev/ttyACM0 scale --mass-g 500

  # step through the whole thing, prompting between stages
  force_sensor_calibrate.py udp:127.0.0.1:14550 calibrate --mass-g 500
'''

import argparse
import sys
import time

from pymavlink import mavutil

# operations for param2 of MAV_CMD_FORCE_SENSOR_CALIBRATE
OP_TARE = 0
OP_SCALE = 1

# the vehicle averages a block of samples before answering; at the slowest
# sample rate that takes a few seconds, and it gives up after ten
OUTCOME_TIMEOUT = 15


class ForceSensorCalibrator(object):
    def __init__(self, connection_string, instance, source_system):
        self.instance = instance
        self.mav = mavutil.mavlink_connection(connection_string,
                                              source_system=source_system,
                                              source_component=190)
        self.wait_vehicle_heartbeat()

    def wait_vehicle_heartbeat(self, timeout=30):
        '''wait for a heartbeat from a vehicle.

        Anything which forwards MAVLink (MAVProxy, mavlink-router) also emits
        its own GCS heartbeat, and latching onto that leaves us addressing the
        forwarder rather than the vehicle.
        '''
        deadline = time.time() + timeout
        while time.time() < deadline:
            m = self.mav.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if m is None:
                continue
            if m.type == mavutil.mavlink.MAV_TYPE_GCS:
                continue
            if m.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                continue
            self.mav.target_system = m.get_srcSystem()
            self.mav.target_component = m.get_srcComponent()
            # look like a GCS ourselves
            self.mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                        0, 0, 0)
            return
        raise Exception("No vehicle heartbeat received")

    def get_param(self, name, timeout=5):
        '''fetch a parameter, or None if the vehicle does not have it'''
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.mav.mav.param_request_read_send(
                self.mav.target_system, self.mav.target_component,
                name.encode('ascii'), -1)
            sub_deadline = time.time() + 1
            while time.time() < sub_deadline:
                m = self.mav.recv_match(type='PARAM_VALUE', blocking=True,
                                        timeout=0.3)
                if m is not None and m.param_id == name:
                    return m.param_value
        return None

    def send_calibrate(self, operation, mass_kg=0):
        '''send the calibration command; returns the MAV_RESULT'''
        self.mav.mav.command_int_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL,
            mavutil.mavlink.MAV_CMD_FORCE_SENSOR_CALIBRATE,
            0,  # current
            0,  # autocontinue
            self.instance,  # param1: instance, numbered from 1
            operation,      # param2: 0=tare, 1=scale
            mass_kg,        # param3: known mass (kg)
            0, 0, 0, 0)

        deadline = time.time() + 10
        while time.time() < deadline:
            m = self.mav.recv_match(type='COMMAND_ACK', blocking=True,
                                    timeout=1)
            if m is None:
                continue
            if m.command != mavutil.mavlink.MAV_CMD_FORCE_SENSOR_CALIBRATE:
                continue
            return m.result
        raise Exception("No COMMAND_ACK received; "
                        "does this firmware support force sensors?")

    def wait_outcome(self, timeout=OUTCOME_TIMEOUT):
        '''wait for the vehicle to report how the calibration went.

        The command is only acknowledged as *started* - the vehicle averages
        samples before it knows whether it worked.
        '''
        deadline = time.time() + timeout
        while time.time() < deadline:
            m = self.mav.recv_match(type='STATUSTEXT', blocking=True,
                                    timeout=1)
            if m is None:
                continue
            text = m.text
            if 'ForceSensor' not in text:
                continue
            print("  %s" % text)
            if 'failed' in text:
                return False
            if 'tared' in text or 'calibrated' in text:
                return True
        print("  timed out waiting for the vehicle to report the outcome")
        return False

    def run_operation(self, operation, mass_kg=0):
        '''send an operation and report on it; returns True on success'''
        result = self.send_calibrate(operation, mass_kg)
        if result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("Command rejected: %s" % result_name(result))
            if result == mavutil.mavlink.MAV_RESULT_DENIED:
                print("  is FSCL%u_TYPE set, and the instance in range?"
                      % self.instance)
            elif result == mavutil.mavlink.MAV_RESULT_FAILED:
                print("  check that FSCL%u_TYPE is set, that the sensor was "
                      "detected," % self.instance)
                print("  that the vehicle is disarmed, and that a calibration "
                      "is not already running")
            return False
        return self.wait_outcome()

    def show_status(self):
        '''print what is configured and how it is calibrated'''
        found = False
        for i in range(1, 3):
            sensor_type = self.get_param('FSCL%u_TYPE' % i)
            if sensor_type is None or int(sensor_type) == 0:
                continue
            found = True
            print("FSCL%u_TYPE  = %u (%s)"
                  % (i, sensor_type, type_name(int(sensor_type))))
            # these only exist once a backend has actually been allocated
            zero = self.get_param('FSCL%u_ZERO' % i)
            scale = self.get_param('FSCL%u_SCALE' % i)
            if zero is None or scale is None:
                print("  not detected: no backend was allocated for it")
                continue
            print("  FSCL%u_ZERO  = %.1f counts" % (i, zero))
            if scale == 0:
                print("  FSCL%u_SCALE = 0 -> uncalibrated, reporting raw counts"
                      % i)
            else:
                print("  FSCL%u_SCALE = %.1f counts/N" % (i, scale))
        if not found:
            print("No force sensors are configured (set FSCL1_TYPE)")


def type_name(value):
    return {0: 'None', 1: 'NAU7802', 2: 'DroneCAN'}.get(value, 'unknown')


def result_name(value):
    return mavutil.mavlink.enums['MAV_RESULT'][value].name


def prompt(text):
    try:
        input("%s, then press Enter..." % text)
    except (EOFError, KeyboardInterrupt):
        print()
        sys.exit(1)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('connection',
                        help='MAVLink connection string, eg /dev/ttyACM0, '
                             'udp:127.0.0.1:14550, tcp:localhost:5760')
    parser.add_argument('--instance', type=int, default=1,
                        help='force sensor instance, numbered from 1 '
                             '(default 1, matching FSCL1_)')
    parser.add_argument('--source-system', type=int, default=254,
                        help='MAVLink source system to use (default 254; '
                             'MAVProxy defaults to 255)')

    subparsers = parser.add_subparsers(dest='command', required=True)
    subparsers.add_parser('status',
                          help='show configured sensors and their calibration')
    subparsers.add_parser('tare',
                          help='zero the sensor; nothing may be on it')
    for name, help_text in (('scale', 'set scale factor from a mass already '
                                      'applied to the sensor'),
                            ('calibrate', 'zero, then set the scale factor, '
                                          'prompting between stages')):
        sub = subparsers.add_parser(name, help=help_text)
        group = sub.add_mutually_exclusive_group(required=True)
        group.add_argument('--mass-kg', type=float, help='known mass, in kg')
        group.add_argument('--mass-g', type=float, help='known mass, in grams')

    args = parser.parse_args()

    mass_kg = 0
    if getattr(args, 'mass_g', None) is not None:
        mass_kg = args.mass_g / 1000.0
    elif getattr(args, 'mass_kg', None) is not None:
        mass_kg = args.mass_kg
    if args.command in ('scale', 'calibrate') and mass_kg <= 0:
        print("The calibration mass must be greater than zero")
        return 1

    calibrator = ForceSensorCalibrator(args.connection, args.instance,
                                       args.source_system)

    if args.command == 'status':
        calibrator.show_status()
        return 0

    if args.command == 'tare':
        print("Zeroing sensor %u..." % args.instance)
        return 0 if calibrator.run_operation(OP_TARE) else 1

    if args.command == 'scale':
        print("Calibrating sensor %u against %.3f kg..."
              % (args.instance, mass_kg))
        return 0 if calibrator.run_operation(OP_SCALE, mass_kg) else 1

    # full sequence
    print("Calibrating force sensor %u against %.3f kg" %
          (args.instance, mass_kg))
    prompt("Remove everything from the sensor")
    print("Zeroing...")
    if not calibrator.run_operation(OP_TARE):
        return 1
    prompt("Place the %.3f kg on the sensor" % mass_kg)
    print("Measuring...")
    if not calibrator.run_operation(OP_SCALE, mass_kg):
        return 1
    print("Done. The sensor now reports force in Newtons.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
