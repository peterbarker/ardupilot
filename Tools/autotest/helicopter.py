'''
Fly Helicopter in SITL

AP_FLAKE8_CLEAN
'''

import math

from pymavlink import mavutil

import vehicle_test_suite

from arducopter import AutoTestCopter
from vehicle_test_suite import AltFrame
from vehicle_test_suite import Location
from vehicle_test_suite import NotAchievedException


class AutoTestHelicopter(AutoTestCopter):

    test_root = "Helicopter_Tests"
    sitl_start_loc = Location(40.072842, -105.230575, 1586, AltFrame.ABSOLUTE)  # Sparkfun AVC Location
    sitl_start_heading_deg = 0

    def max_distance_from_startup_location_at_end_of_test(self):
        # this class inherits ArduCopter's tests but not its
        # discipline of leaving the vehicle where it started: a heli
        # which takes off and lands drifts further than a multirotor
        # (CI measured 2.5m and 3.1m for the takeoff tests), and the
        # autorotation tests deliberately end well away from the
        # startup location (110m).
        return None

    def vehicleinfo_key(self):
        return 'Helicopter'

    def log_name(self):
        return "HeliCopter"

    def default_frame(self):
        return "heli"

    def sitl_start_location(self):
        return self.sitl_start_loc

    def sitl_start_heading(self):
        return self.sitl_start_heading_deg

    def is_heli(self):
        return True

    def subgroupvarptr_activation_params(self):
        ret = super(AutoTestHelicopter, self).subgroupvarptr_activation_params()
        # AC_CustomControl is disabled on heli (AC_CUSTOMCONTROL_MULTI_ENABLED is false)
        ret.pop("CC_TYPE", None)
        return ret

    def rc_defaults(self):
        ret = super(AutoTestHelicopter, self).rc_defaults()
        ret[8] = 1000
        ret[3] = 1000 # collective
        return ret

    @staticmethod
    def get_position_armable_modes_list():
        '''filter THROW mode out of armable modes list; Heli is special-cased'''
        ret = AutoTestCopter.get_position_armable_modes_list()
        ret = filter(lambda x : x != "THROW", ret)
        return ret

    def loiter_requires_position(self):
        self.progress("Skipping loiter-requires-position for heli; rotor runup issues")

    def get_collective_out(self):
        servo = self.assert_receive_message('SERVO_OUTPUT_RAW')
        chan_pwm = (servo.servo1_raw + servo.servo2_raw + servo.servo3_raw)/3.0
        return chan_pwm

    def takeoff(self,
                altitude_min=30,
                takeoff_throttle=1700,
                require_absolute=True,
                mode="STABILIZE",
                timeout=120,
                altitude_max=None):
        """Takeoff to at least altitude_min metres above home.

        Beware: in a manual-collective mode such as STABILIZE the
        vehicle can blow way past altitude_min before the collective is
        reduced, and unless altitude_max is supplied nothing checks the
        overshoot.  If your test cares about the altitude the takeoff
        finishes at, take off in GUIDED.
        """
        self.progress("TAKEOFF")
        self.change_mode(mode)
        if not self.armed():
            self.wait_ready_to_arm(require_absolute=require_absolute, timeout=timeout)
            self.zero_throttle()
            self.arm_vehicle()

        self.progress("Raising rotor speed")
        self.set_rc(8, 2000)
        self.progress("wait for rotor runup to complete")
        if self.get_parameter("H_RSC_MODE") == 4:
            self.context_collect('STATUSTEXT')
            self.wait_statustext("Runup Complete", check_context=True)
        elif self.get_parameter("H_RSC_MODE") == 3:
            self.wait_rpm(1, 1300, 1400)
        else:
            self.wait_servo_channel_value(8, 1659, timeout=10)

        # wait for motor runup
        self.delay_sim_time(20, reason="rotor runup to complete")

        if mode == 'GUIDED':
            max_err = 5
            if altitude_max is not None:
                max_err = altitude_max - altitude_min
            self.user_takeoff(alt_min=altitude_min, max_err=max_err)
        else:
            self.set_rc(3, takeoff_throttle)
        if altitude_max is None:
            # no limit; a finite stand-in as wait_and_maintain does
            # arithmetic on the bounds
            altitude_max = 100000
        self.wait_altitude(altitude_min-1, altitude_max, relative=True, timeout=timeout)
        self.hover()
        self.progress("TAKEOFF COMPLETE")

    def fly_inverted_flight(self, collective_servos=None, yaw_tolerance=20):
        '''engage inverted flight from a hover in ALT_HOLD and verify that the
        vehicle stays inverted and holds altitude, then recover.  Assumes the
        frame has already been set up and the vehicle is disarmed.  If
        collective_servos is given those servo channels must reach negative
        blade pitch (PWM below mid) while inverted.  yaw_tolerance is the
        allowed heading drift in degrees, or None to not check heading (a
        single tail rotor re-trims when inverted and drifts slowly).'''
        rc_option_inverted = 43
        self.set_parameter("RC10_OPTION", rc_option_inverted)
        self.set_rc(10, 1000)
        # clear any interlock/throttle override left over from a previous
        # frame's attempt, so the vehicle can pass its arming checks
        self.zero_throttle()
        self.set_rc(8, 1000)
        self.takeoff(30, mode='ALT_HOLD')

        self.progress("Engaging inverted flight")
        self.set_rc(10, 2000)
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 15:
                raise NotAchievedException("Did not roll inverted")
            m = self.assert_receive_message('ATTITUDE')
            if abs(math.degrees(m.roll)) > 150:
                break

        self.progress("Holding inverted flight")
        hold_alt = self.get_altitude(relative=True)
        m = self.assert_receive_message('ATTITUDE')
        hold_yaw_deg = math.degrees(m.yaw)
        max_collective = 1000
        tstart = self.get_sim_time()
        while self.get_sim_time_cached() - tstart < 10:
            m = self.assert_receive_message('ATTITUDE')
            roll_deg = math.degrees(m.roll)
            if abs(roll_deg) < 150:
                raise NotAchievedException(f"Did not stay inverted (roll {roll_deg:.1f})")
            if yaw_tolerance is not None:
                yaw_err = (math.degrees(m.yaw) - hold_yaw_deg + 180) % 360 - 180
                if abs(yaw_err) > yaw_tolerance:
                    raise NotAchievedException(f"Did not hold heading while inverted (err {yaw_err:.1f}deg)")
            alt = self.get_altitude(relative=True)
            if abs(alt - hold_alt) > 5:
                raise NotAchievedException(f"Lost altitude while inverted ({alt:.1f}m vs {hold_alt:.1f}m)")
            if collective_servos is not None:
                servo = self.assert_receive_message('SERVO_OUTPUT_RAW')
                for n in collective_servos:
                    max_collective = max(max_collective, getattr(servo, f"servo{n}_raw"))
        if collective_servos is not None and max_collective >= 1500:
            raise NotAchievedException("Rotor not at negative collective while inverted (max %u)" % max_collective)
        self.progress("Inverted flight held altitude and heading")

        self.progress("Disengaging inverted flight")
        self.set_rc(10, 1000)
        self.wait_attitude(desroll=0, despitch=0, tolerance=10, timeout=15)

    def hover(self):
        self.progress("Setting hover collective")
        self.set_rc(3, 1500)

    def mission_item_rtl(self, target_system, target_component):
        '''returns a mission_item_int which can be used as takeoff in a mission'''
        return self.mav.mav.mission_item_int_encode(
            target_system,
            target_component,
            1, # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0, # current
            0, # autocontinue
            0, # p1
            0, # p2
            0, # p3
            0, # p4
            0, # latitude
            0, # longitude
            0.0000, # altitude
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION)

    def set_rc_default(self):
        super(AutoTestHelicopter, self).set_rc_default()
        self.progress("Lowering rotor speed")
        self.set_rc(8, 1000)

    def fly_mission(self, filename, strict=True):
        num_wp = self.load_mission(filename, strict=strict)
        self.change_mode("LOITER")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.set_rc(8, 2000)    # Raise rotor speed
        self.delay_sim_time(20, reason="rotor to reach speed")
        self.change_mode("AUTO")
        self.set_rc(3, 1500)

        self.wait_waypoint(1, num_wp-1)
        self.wait_disarmed()
        self.set_rc(3, 1000)
        self.set_rc(8, 1000)    # Lower rotor speed

    def land_and_disarm(self, **kwargs):
        super(AutoTestHelicopter, self).land_and_disarm(**kwargs)
        self.progress("Killing rotor speed")
        self.set_rc(8, 1000)

    def do_RTL(self, **kwargs):
        super(AutoTestHelicopter, self).do_RTL(**kwargs)
        self.progress("Killing rotor speed")
        self.set_rc(8, 1000)

    def tests(self):
        '''return list of all tests'''
        ret = vehicle_test_suite.TestSuite.tests(self)
        ret.extend(self.discovered_tests("Helicopter"))
        return ret

    def disabled_tests(self):
        return {
        }
