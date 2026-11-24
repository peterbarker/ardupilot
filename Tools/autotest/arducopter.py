'''
Fly Copter in SITL

AP_FLAKE8_CLEAN
'''

import copy
import math
import os
import shutil
import struct

import numpy

from pymavlink import mavextra
from pymavlink import mavutil
from pymavlink import quaternion
from pymavlink import rotmat
from pymavlink.rotmat import Matrix3

import vehicle_test_suite

from pysim import util
from vehicle_test_suite import MAV_POS_TARGET_TYPE_MASK
from vehicle_test_suite import AltFrame
from vehicle_test_suite import AutoTestTimeoutException
from vehicle_test_suite import Location
from vehicle_test_suite import NotAchievedException
from vehicle_test_suite import PreconditionFailedException

# CAMERA_CAPTURE_STATUS.image_status is documented in the field description
# rather than in an enumeration, so give the values we use names here
CAMERA_IMAGE_STATUS_IDLE = 0
CAMERA_IMAGE_STATUS_INTERVAL_IDLE = 2

# get location of scripts
testdir = os.path.dirname(os.path.realpath(__file__))
SITL_START_LOCATION = Location(
    -35.362938,
    149.165085,
    584.0805053710938,
    AltFrame.ABSOLUTE,
)
SITL_START_HEADING = 270

# Flight mode switch positions are set-up in arducopter.param to be
#   switch 1 = Circle
#   switch 2 = Land
#   switch 3 = RTL
#   switch 4 = Auto
#   switch 5 = Loiter
#   switch 6 = Stabilize


class AutoTestCopter(vehicle_test_suite.TestSuite):
    @staticmethod
    def get_not_armable_mode_list():
        return ["AUTO", "AUTOTUNE", "BRAKE", "CIRCLE", "FLIP", "LAND", "RTL", "SMART_RTL", "AVOID_ADSB", "FOLLOW"]

    @staticmethod
    def get_not_disarmed_settable_modes_list():
        return ["FLIP", "AUTOTUNE"]

    @staticmethod
    def get_no_position_not_settable_modes_list():
        return []

    @staticmethod
    def get_position_armable_modes_list():
        return ["DRIFT", "GUIDED", "LOITER", "POSHOLD", "THROW"]

    @staticmethod
    def get_normal_armable_modes_list():
        return ["ACRO", "ALT_HOLD", "STABILIZE", "GUIDED_NOGPS"]

    def log_name(self):
        return "ArduCopter"

    def test_filepath(self):
        return os.path.realpath(__file__)

    test_root = "ArduCopter_Tests"

    def sitl_start_location(self):
        return SITL_START_LOCATION

    def sitl_start_heading(self):
        return SITL_START_HEADING

    def max_distance_from_startup_location_at_end_of_test(self):
        # Copter's tests start from the point the simulation puts the
        # vehicle, so each of them has to leave it there.  Two metres
        # sits in the gap the measurements show: a copter which takes
        # off and lands comes back to within about 1.3m of where it
        # started - Landing measured 1.24m and 1.10m on consecutive
        # runs - while the tests which genuinely fly away and stay away
        # start at 2.38m and run to 400m.
        return 2

    def mavproxy_options(self):
        ret = super(AutoTestCopter, self).mavproxy_options()
        if self.frame != 'heli':
            ret.append('--quadcopter')
        return ret

    def sitl_streamrate(self):
        return 5

    def vehicleinfo_key(self):
        return 'ArduCopter'

    def default_frame(self):
        return "+"

    def callisto_sitl_kwargs(self):
        """Returns kwargs for a SITL commandline to fly the Callisto. Wipes params."""
        return {
            "model": "octa-quad-cwx:@ROMFS/models/Callisto.json",
            "wipe": True
        }

    def wait_disarmed_default_wait_time(self):
        return 120

    def close(self):
        super(AutoTestCopter, self).close()

        # [2014/05/07] FC Because I'm doing a cross machine build
        # (source is on host, build is on guest VM) I cannot hard link
        # This flag tells me that I need to copy the data out
        if self.copy_tlog:
            shutil.copy(self.logfile, self.buildlog)

    def is_copter(self):
        return True

    def get_stick_arming_channel(self):
        return int(self.get_parameter("RCMAP_YAW"))

    def get_disarm_delay(self):
        return int(self.get_parameter("DISARM_DELAY"))

    def set_autodisarm_delay(self, delay):
        self.set_parameter("DISARM_DELAY", delay)

    def takeoff(self,
                altitude_min=30,
                takeoff_throttle=1700,
                require_absolute=True,
                mode="STABILIZE",
                timeout=120,
                altitude_max=None,
                alt_minimum_duration=0,
                ):
        """Takeoff to at least altitude_min metres above home.

        Beware: in a manual-throttle mode such as STABILIZE the vehicle
        climbs fast and can blow way past altitude_min before the
        throttle is reduced, and unless altitude_max is supplied
        nothing checks the overshoot.  If your test cares about the
        altitude the takeoff finishes at, take off in GUIDED.
        """
        self.progress("TAKEOFF")
        self.change_mode(mode)
        if not self.armed():
            self.wait_ready_to_arm(require_absolute=require_absolute, timeout=timeout)
            if mode != 'GUIDED':
                # no RC fiddling in GUIDED; tests with a dead or
                # ignored RC receiver rely on a GUIDED takeoff working
                # without any RC input at all
                self.zero_throttle()
            self.arm_vehicle()
        if mode == 'GUIDED':
            max_err = 5
            if altitude_max is not None:
                max_err = altitude_max - altitude_min
            self.user_takeoff(alt_min=altitude_min, timeout=timeout, max_err=max_err)
        else:
            self.set_rc(3, takeoff_throttle)
        if altitude_max is None:
            # no limit; a finite stand-in as wait_and_maintain does
            # arithmetic on the bounds
            altitude_max = 100000
        self.wait_altitude(altitude_min-1, altitude_max, relative=True, timeout=timeout, minimum_duration=alt_minimum_duration)
        if mode != 'GUIDED':
            self.hover()
        self.progress("TAKEOFF COMPLETE")

    def land_and_disarm(self, timeout=60):
        """Land the quad."""
        self.progress("STARTING LANDING")
        self.change_mode("LAND")
        self.wait_landed_and_disarmed(timeout=timeout)

    def wait_landed_and_disarmed(self, min_alt=6, timeout=60):
        """Wait to be landed and disarmed"""
        m = self.assert_receive_message('GLOBAL_POSITION_INT')
        alt = m.relative_alt / 1000.0 # mm -> m
        if alt > min_alt:
            self.wait_altitude(min_alt-1, min_alt+5, relative=True, timeout=timeout)
#        self.wait_statustext("SIM Hit ground", timeout=timeout)
        self.wait_disarmed()

    def hover(self, hover_throttle=1500):
        self.set_rc(3, hover_throttle)

    # Climb/descend to a given altitude
    def setAlt(self, desiredAlt=50):
        pos = self.get_location(frame=AltFrame.ABOVE_HOME)
        pos_alt = pos.get_alt_m(AltFrame.ABOVE_HOME)
        if pos_alt > desiredAlt:
            self.set_rc(3, 1300)
            self.wait_altitude((desiredAlt-5), desiredAlt, relative=True)
        if pos_alt < (desiredAlt-5):
            self.set_rc(3, 1800)
            self.wait_altitude((desiredAlt-5), desiredAlt, relative=True)
        self.hover()

    # Takeoff, climb to given altitude, and fly east for 10 seconds
    def takeoffAndMoveAway(self, dAlt=50, dDist=50):
        self.progress("Centering sticks")
        self.set_rc_from_map({
            1: 1500,
            2: 1500,
            3: 1000,
            4: 1500,
        })
        self.takeoff(altitude_min=dAlt, mode='GUIDED')
        self.hover()
        self.change_mode("ALT_HOLD")

        self.progress("Yaw to east")
        self.set_rc(4, 1580)
        self.wait_heading(90)
        self.set_rc(4, 1500)

        self.progress("Fly eastbound away from home")
        self.set_rc(2, 1800)
        self.delay_sim_time(10, reason="copter to fly east")
        self.set_rc(2, 1500)
        self.hover()
        self.progress("Copter staging 50 meters east of home at 50 meters altitude In mode Alt Hold")

    def fly_to_origin(self, final_alt=10):
        origin = self.poll_message("GPS_GLOBAL_ORIGIN")
        self.change_mode("GUIDED")
        self.guided_move_global_relative_alt(origin.latitude,
                                             origin.longitude,
                                             final_alt)

    def change_alt(self, alt_min, *,
                   threshold_below_m=5.0, threshold_above_m=0.0,
                   climb_throttle=1920, descend_throttle=1080):
        """Change altitude."""
        def adjust_altitude(current_alt, target_alt, accuracy):
            if math.fabs(current_alt - target_alt) <= accuracy:
                self.hover()
            elif current_alt < target_alt:
                self.set_rc(3, climb_throttle)
            else:
                self.set_rc(3, descend_throttle)
        self.wait_altitude(
            (alt_min - threshold_below_m),
            (alt_min + threshold_above_m),
            relative=True,
            called_function=lambda current_alt, target_alt: adjust_altitude(current_alt, target_alt, 1)
        )
        self.hover()

    def circle_from_arc(self, p1, p2, theta_deg):
        """
        Compute a single circle center and radius given two points on a circle
        and the signed central angle they subtend.

        p1, p2: (x, y)
        theta: signed central angle in radians
               - the absolute value is used for geometry
               - the sign determines which side of the chord the center is on

        ... with thanks to ChatGPT

        """

        theta = math.radians(theta_deg)

        (x1, y1), (x2, y2) = p1, p2

        # chord vector and length
        dx = x2 - x1
        dy = y2 - y1
        d = math.hypot(dx, dy)

        # radius from absolute angle
        a = abs(theta)
        R = d / (2 * math.sin(a / 2))

        # midpoint of chord
        mx = (x1 + x2) / 2
        my = (y1 + y2) / 2

        # distance from midpoint to center
        h = math.sqrt(R**2 - (d/2)**2)

        # unit perpendicular
        ux = -dy / d
        uy = dx / d

        # choose side based on sign of theta
        sign = 1 if theta >= 0 else -1

        cx = mx + sign * h * ux
        cy = my + sign * h * uy

        return (cx, cy), R

    # enter RTL mode and wait for the vehicle to disarm
    def do_RTL(self, distance_min=None, check_alt=True, alt_max=1, distance_max=10, timeout=250, quiet=False):
        """Enter RTL mode and wait for the vehicle to disarm at Home."""
        self.change_mode("RTL")
        self.zero_throttle()
        self.wait_rtl_complete(check_alt=check_alt, alt_max=alt_max, distance_max=distance_max, timeout=timeout, quiet=True)

    def wait_rtl_complete(self, check_alt=True, alt_max=1, distance_max=10, timeout=250, quiet=False):
        """Wait for RTL to reach home and disarm"""
        self.progress("Waiting RTL to reach Home and disarm")
        tstart = self.get_sim_time()
        while self.get_sim_time_cached() < tstart + timeout:
            m = self.assert_receive_message('GLOBAL_POSITION_INT')
            alt = m.relative_alt / 1000.0 # mm -> m
            home_distance = self.distance_to_home(use_cached_home=True)
            home = ""
            alt_valid = alt <= alt_max
            distance_valid = home_distance < distance_max
            if check_alt:
                if alt_valid and distance_valid:
                    home = "HOME"
            else:
                if distance_valid:
                    home = "HOME"
            if not quiet:
                self.progress("Alt: %.02f  HomeDist: %.02f %s" %
                              (alt, home_distance, home))

            # our post-condition is that we are disarmed:
            if not self.armed(cached=True):
                if home == "":
                    raise NotAchievedException("Did not get home")
                # success!
                return

        raise AutoTestTimeoutException("Did not get home and disarm")

    # Tests all actions and logic behind the battery failsafe
    def batt_failsafe_init(self):
        '''configure the common battery failsafe parameters shared by the
        individual BatteryFailsafe* tests'''
        self.progress("Configure battery failsafe parameters")
        self.set_parameters({
            'BATT_LOW_VOLT': 11.5,
            'BATT_CRT_VOLT': 10.1,
            'BATT_FS_LOW_ACT': 0,
            'BATT_FS_CRT_ACT': 0,
            'FS_OPTIONS': 0,
            'SIM_BATT_VOLTAGE': 12.5,
        })

    def clear_battery_failsafe(self):
        '''clears battery failsafe (and resets % to 100)'''
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_BATTERY_RESET,
            p1=65535,   # battery mask
            p2=100,      # percentage
        )

    def assert_dataflash_message_field_level_at(self,
                                                mtype,
                                                field,
                                                level,
                                                maintain=1,
                                                tolerance=0.05,
                                                timeout=30,
                                                condition=None,
                                                dfreader_start_timestamp=None,
                                                verbose=False):
        '''wait for EKF's accel bias to reach a level and maintain it'''

        if verbose:
            self.progress("current onboard log filepath: %s" % self.current_onboard_log_filepath())
        dfreader = self.dfreader_for_current_onboard_log()

        achieve_start = None
        current_value = None
        while True:
            m = dfreader.recv_match(type=mtype, condition=condition)
            if m is None:
                raise NotAchievedException("%s.%s did not maintain %f" %
                                           (mtype, field, level))
            if dfreader_start_timestamp is not None:
                if m.TimeUS < dfreader_start_timestamp:
                    continue
            if verbose:
                print("m=%s" % str(m))
            current_value = getattr(m, field)

            if abs(current_value - level) > tolerance:
                if achieve_start is not None:
                    self.progress("Achieve stop at %u" % m.TimeUS)
                    achieve_start = None
                continue

            dfreader_now = m.TimeUS
            if achieve_start is None:
                self.progress("Achieve start at %u (got=%f want=%f)" %
                              (dfreader_now, current_value, level))
                if maintain is None:
                    return
                achieve_start = m.TimeUS
                continue

            # we're achieving....
            if dfreader_now - achieve_start > maintain*1e6:
                return dfreader_now

    def debug_arming_issue(self):
        while True:
            self.send_mavlink_arm_command()
            m = self.mav.recv_match(blocking=True, timeout=1)
            if m is None:
                continue
            if m.get_type() in ["STATUSTEXT", "COMMAND_ACK"]:
                print("Got: %s" % str(m))
            if self.mav.motors_armed():
                self.progress("Armed")
                return

    # fly_fence_test - fly east until you hit the horizontal circular fence
    avoid_behave_slide = 0

    def fly_fence_avoid_test_radius_check(self, timeout=180, avoid_behave=avoid_behave_slide):
        using_mode = "LOITER" # must be something which adjusts velocity!
        self.change_mode(using_mode)
        fence_radius = 15
        fence_margin = 3
        self.set_parameters({
            "FENCE_ENABLE": 1, # fence
            "FENCE_TYPE": 2, # circle
            "FENCE_RADIUS": fence_radius,
            "FENCE_MARGIN": fence_margin,
            "AVOID_ENABLE": 1,
            "AVOID_BEHAVE": avoid_behave,
            "RC10_OPTION": 40, # avoid-enable
        })
        self.wait_ready_to_arm()
        self.set_rc(10, 2000)
        home_distance = self.distance_to_home(use_cached_home=True)
        if home_distance > 5:
            raise PreconditionFailedException("Expected to be within 5m of home")
        self.zero_throttle()
        self.arm_vehicle()
        self.set_rc(3, 1700)
        self.wait_altitude(10, 100, relative=True)
        self.set_rc_from_map({3: 1500, 2: 1400})
        self.wait_distance_to_home(12, 20, timeout=30)
        tstart = self.get_sim_time()
        push_time = 70 # push against barrier for 60 seconds
        failed_max = False
        failed_min = False
        while True:
            if self.get_sim_time() - tstart > push_time:
                self.progress("Push time up")
                break
            # make sure we don't RTL:
            if not self.mode_is(using_mode, poll=False):
                raise NotAchievedException("Changed mode away from %s" % using_mode)
            distance = self.distance_to_home(use_cached_home=True)
            inner_radius = fence_radius - fence_margin
            want_min = inner_radius - 1 # allow 1m either way
            want_max = inner_radius + 1 # allow 1m either way
            self.progress("Push: distance=%f %f<want<%f" %
                          (distance, want_min, want_max))
            if distance < want_min:
                if failed_min is False:
                    self.progress("Failed min")
                    failed_min = True
            if distance > want_max:
                if failed_max is False:
                    self.progress("Failed max")
                    failed_max = True
        if failed_min and failed_max:
            raise NotAchievedException("Failed both min and max checks.  Clever")
        if failed_min:
            raise NotAchievedException("Failed min")
        if failed_max:
            raise NotAchievedException("Failed max")
        self.set_rc(2, 1500)
        self.do_RTL()

    # values for SIM_GPSn_UBXMDL
    UBX_MODEL_UBLOX6 = 1
    UBX_MODEL_M8N_FW201 = 2
    UBX_MODEL_M8_ROM301 = 3
    UBX_MODEL_M8N_SPG301 = 4
    UBX_MODEL_M9N_SPG404 = 5
    UBX_MODEL_M10_SPG510 = 6

    def ublox_sim_config_filepath(self):
        '''the file the simulated u-blox keeps its saved configuration in'''
        return os.path.join(os.getcwd(), "sim_ublox_gps1.bin")

    def ublox_sim_reset_saved_config(self):
        '''return the simulated u-blox to factory configuration at next boot'''
        filepath = self.ublox_sim_config_filepath()
        if os.path.exists(filepath):
            os.unlink(filepath)

    # offset of the NAV-SOL rate in the simulator's saved configuration
    # file: magic(4), model(1), then the rates of the tracked messages
    UBX_SIM_NAV_SOL_RATE_OFFSET = 5 + 3

    def ublox_sim_saved_nav_sol_rate(self):
        '''return the NAV-SOL rate in the simulated u-blox's saved configuration'''
        with open(self.ublox_sim_config_filepath(), "rb") as f:
            data = f.read()
        (magic, ) = struct.unpack("<I", data[0:4])
        if magic != 0x55425853:
            raise NotAchievedException("Bad magic in saved u-blox config")
        return data[self.UBX_SIM_NAV_SOL_RATE_OFFSET]

    # offset of CFG-SIGNAL-BDS_B1C_ENA in the saved configuration file:
    # after the rates come measure/nav rate and timeref (6), NAV5 (36),
    # SBAS (8), GNSS length and settings (1+60), then the
    # CFG-VALSET keys, of which this is the sixth
    UBX_SIM_BDS_B1C_ENA_OFFSET = 5 + 14 + 6 + 36 + 8 + 1 + 60 + 5

    def ublox_sim_boot(self, settle_time=40):
        '''reboot, wait for the u-blox to be detected and configured,
        return the text of the STATUSTEXTs received'''
        self.context_clear_collection('STATUSTEXT')
        self.reboot_sitl()
        self.wait_statustext("detected u-blox", check_context=True, timeout=60)
        self.delay_sim_time(settle_time, reason="let the u-blox driver configure the receiver")
        texts = [m.text for m in self.context_collection('STATUSTEXT')]
        counts = {}
        for text in texts:
            if "u-blox" in text:
                counts[text] = counts.get(text, 0) + 1
        for (text, count) in counts.items():
            self.progress("  %ux %s" % (count, text))
        return texts

    def ublox_sim_count_saves(self, texts):
        return len([t for t in texts if "u-blox saving config" in t])

    def configure_EKFs_to_use_optical_flow_instead_of_GPS(self):
        '''configure EKF to use optical flow instead of GPS'''
        ahrs_ekf_type = self.get_parameter("AHRS_EKF_TYPE")
        if ahrs_ekf_type == 2:
            self.set_parameter("EK2_GPS_TYPE", 3)
        if ahrs_ekf_type == 3:
            self.set_parameters({
                "EK3_SRC1_POSXY": 0,
                "EK3_SRC1_VELXY": 5,
                "EK3_SRC1_VELZ": 0,
            })

    def set_origin(self, loc, timeout=60):
        '''set the GPS global origin to loc'''
        self.progress("Setting origin")
        if isinstance(loc, mavutil.mavlink.MAVLink_global_position_int_message):
            lat_1e7 = loc.lat
            lng_1e7 = loc.lon
            alt_mm = loc.alt
        else:
            # assume a Location; the origin's altitude is AMSL
            lat_1e7 = int(loc.lat * 1e7)
            lng_1e7 = int(loc.lng * 1e7)
            alt_mm = int(loc.get_alt_m(AltFrame.ABSOLUTE) * 1e3)

        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > timeout:
                raise AutoTestTimeoutException("Did not get non-zero lat")
            target_system = 1
            self.mav.mav.set_gps_global_origin_send(
                target_system,
                lat_1e7,
                lng_1e7,
                alt_mm
            )
            self.delay_sim_time(2, reason="GPS origin to propagate")
            gpi = self.assert_receive_message('GLOBAL_POSITION_INT')
            self.progress("gpi=%s" % str(gpi))
            if gpi.lat != 0:
                break

    def fly_loaded_mission(self, num_wp):
        '''fly mission loaded on vehicle.  FIXME: get num_wp from vehicle'''
        self.progress("test: Fly a mission from 1 to %u" % num_wp)
        self.set_current_waypoint(1)

        self.change_mode("LOITER")
        self.wait_ready_to_arm()
        self.arm_vehicle()

        # switch into AUTO mode and raise throttle
        self.change_mode("AUTO")
        self.set_rc(3, 1500)

        # fly the mission
        self.wait_waypoint(0, num_wp-1, timeout=500)

        # set throttle to minimum
        self.zero_throttle()

        # wait for disarm
        self.wait_disarmed()
        self.progress("MOTORS DISARMED OK")

    def hover_for_interval(self, hover_time):
        '''hovers for an interval of hover_time seconds.  Returns the bookend
        times for that interval (in time-since-boot frame), and the
        output throttle level at the end of the period.
        '''
        self.progress("Hovering for %u seconds" % hover_time)
        tstart = self.get_sim_time()
        self.delay_sim_time(hover_time, reason='data collection')
        vfr_hud = self.poll_message('VFR_HUD')
        tend = self.get_sim_time()
        return tstart, tend, vfr_hud.throttle

    def load_and_start_mission(self, filename, strict=True):
        num_wp = self.load_mission(filename, strict=strict)
        self.set_parameter("AUTO_OPTIONS", 3)
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        return num_wp

    def fly_mission(self, filename, strict=True):
        num_wp = self.load_and_start_mission(filename, strict)
        self.wait_waypoint(num_wp-1, num_wp-1)
        self.wait_disarmed()

    def fly_generic_mission(self, filename, strict=True):
        num_wp = self.load_generic_mission(filename, strict=strict)
        self.set_parameter("AUTO_OPTIONS", 3)
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.wait_waypoint(num_wp-1, num_wp-1)
        self.wait_disarmed()

    def test_rangefinder_switchover(self):
        """test that the EKF correctly handles the switchover between baro and rangefinder"""
        self.set_analog_rangefinder_parameters()

        self.set_parameters({
            "RNGFND1_MAX": 15.00
        })

        # configure EKF to use rangefinder for altitude at low altitudes
        ahrs_ekf_type = self.get_parameter("AHRS_EKF_TYPE")
        if ahrs_ekf_type == 2:
            self.set_parameter("EK2_RNG_USE_HGT", 70)
        if ahrs_ekf_type == 3:
            self.set_parameter("EK3_RNG_USE_HGT", 70)

        self.reboot_sitl() # needed for both rangefinder and initial position
        self.assert_vehicle_location_is_at_startup_location()

        self.change_mode("LOITER")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.set_rc_from_map({3: 1800, 2: 1200})
        # wait till we get to 50m
        self.wait_altitude(50, 52, True, 60)

        self.change_mode("RTL")
        # wait till we get to 25m
        self.wait_altitude(25, 27, True, 120)

        # level up
        self.set_rc(2, 1500)
        self.wait_altitude(14, 15, relative=True)

        self.wait_rtl_complete()

    def get_system_clock_utc(self, time_seconds):
        # this is a copy of ArduPilot's AP_RTC function!
        # separate time into ms, sec, min, hour and days but all expressed
        # in milliseconds
        time_ms = time_seconds * 1000
        ms = time_ms % 1000
        sec_ms = (time_ms % (60 * 1000)) - ms
        min_ms = (time_ms % (60 * 60 * 1000)) - sec_ms - ms
        hour_ms = (time_ms % (24 * 60 * 60 * 1000)) - min_ms - sec_ms - ms

        # convert times as milliseconds into appropriate units
        secs = sec_ms / 1000
        mins = min_ms / (60 * 1000)
        hours = hour_ms / (60 * 60 * 1000)
        return (hours, mins, secs, 0)

    def calc_delay(self, seconds, delay_for_seconds):
        # delay-for-seconds has to be long enough that we're at the
        # waypoint before that time.  Otherwise we'll try to wait a
        # day....
        if delay_for_seconds >= 3600:
            raise ValueError("Won't handle large delays")
        (hours,
         mins,
         secs,
         ms) = self.get_system_clock_utc(seconds)
        self.progress("Now is %uh %um %us" % (hours, mins, secs))
        secs += delay_for_seconds # add seventeen seconds
        mins += int(secs/60)
        secs %= 60

        hours += int(mins / 60)
        mins %= 60

        if hours > 24:
            raise ValueError("Way too big a delay")
        self.progress("Delay until %uh %um %us" %
                      (hours, mins, secs))
        return (hours, mins, secs, 0)

    def reset_delay_item(self, seq, seconds_in_future):
        '''replace mission item seq with a MAV_CMD_NAV_DELAY item delaying
        until seconds_in_future from now'''
        command = mavutil.mavlink.MAV_CMD_NAV_DELAY

        # we have to work out the absolute delay time...
        now = self.mav.messages["SYSTEM_TIME"]
        if now is None:
            raise PreconditionFailedException("Never got SYSTEM_TIME")
        if now.time_unix_usec == 0:
            raise PreconditionFailedException("system time is zero")
        (hours, mins, secs, ms) = self.calc_delay(now.time_unix_usec/1000000, seconds_in_future)

        self.progress("Sending absolute-time mission item")
        item = self.create_MISSION_ITEM_INT(
            command,
            p2=hours,
            p3=mins,
            p4=secs,
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            autocontinue=1,
        )
        self.upload_simple_relhome_mission([item], start_index=seq)

        # retrieve mission item and check it:
        st = self.assert_fetch_mission_item_int(
            1, 1, seq, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        print("Item: %s" % str(st))
        if (st.command != command or
                st.param2 != hours or
                st.param3 != mins or
                st.param4 != secs):
            raise NotAchievedException("Delay item did not update as expected")

    def send_set_position_target_global_int(self, lat, lon, alt):
        self.mav.mav.set_position_target_global_int_send(
            0, # timestamp
            1, # target system_id
            1, # target component id
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            MAV_POS_TARGET_TYPE_MASK.POS_ONLY, # mask specifying use-only-lat-lon-alt
            lat, # lat
            lon, # lon
            alt, # alt
            0, # vx
            0, # vy
            0, # vz
            0, # afx
            0, # afy
            0, # afz
            0, # yaw
            0, # yawrate
        )

    def fly_guided_move_global_relative_alt(self, lat, lon, alt):
        startpos = self.assert_receive_message('GLOBAL_POSITION_INT')

        self.send_set_position_target_global_int(lat, lon, alt)

        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 200:
                raise NotAchievedException("Did not move far enough")
            # send a position-control command
            pos = self.assert_receive_message('GLOBAL_POSITION_INT')
            delta = self.get_distance_int(startpos, pos)
            self.progress("delta=%f (want >10)" % delta)
            if delta > 10:
                break

    def send_position_target_local_ned(self, x, y, z_up):
        self.mav.mav.set_position_target_local_ned_send(
            0, # timestamp
            1, # target system_id
            1, # target component id
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            MAV_POS_TARGET_TYPE_MASK.POS_ONLY | MAV_POS_TARGET_TYPE_MASK.LAST_BYTE, # mask specifying use-only-x-y-z
            x, # x
            y, # y
            -z_up, # z
            0, # vx
            0, # vy
            0, # vz
            0, # afx
            0, # afy
            0, # afz
            0, # yaw
            0, # yawrate
        )

    def fly_guided_move_local(self, x, y, z_up, timeout=100):
        """move the vehicle using MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED"""
        startpos = self.assert_receive_message('LOCAL_POSITION_NED')
        self.progress("startpos=%s" % str(startpos))

        tstart = self.get_sim_time()
        # send a position-control command
        self.send_position_target_local_ned(x, y, z_up)
        while True:
            if self.get_sim_time_cached() - tstart > timeout:
                raise NotAchievedException("Did not reach destination")
            dist = self.distance_to_local_position((x, y, -z_up))
            if dist < 1:
                self.progress(f"Reach distance ({dist})")
                self.progress("endpos=%s" % str(startpos))
                break

    def test_guided_local_velocity_target(self, vx, vy, vz_up, timeout=3):
        " Check local target velocity being received by vehicle "
        self.progress("Setting local NED velocity target: (%f, %f, %f)" % (vx, vy, -vz_up))
        self.progress("Setting POSITION_TARGET_LOCAL_NED message rate to 10Hz")
        self.set_message_rate_hz(mavutil.mavlink.MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED, 10)

        # mask specifying use only vx,vy,vz & accel. Even though we don't test acceltargets below currently
        #  a velocity only mask returns a velocity & accel mask
        target_typemask = (MAV_POS_TARGET_TYPE_MASK.POS_IGNORE |
                           MAV_POS_TARGET_TYPE_MASK.YAW_IGNORE | MAV_POS_TARGET_TYPE_MASK.YAW_RATE_IGNORE)

        # Drain old messages and ignore the ramp-up to the required target velocity
        tstart = self.get_sim_time()
        while self.get_sim_time_cached() - tstart < timeout:
            # send velocity-control command
            self.mav.mav.set_position_target_local_ned_send(
                0, # timestamp
                1, # target system_id
                1, # target component id
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                target_typemask | MAV_POS_TARGET_TYPE_MASK.LAST_BYTE,
                0, # x
                0, # y
                0, # z
                vx, # vx
                vy, # vy
                -vz_up, # vz
                0, # afx
                0, # afy
                0, # afz
                0, # yaw
                0, # yawrate
            )
            m = self.assert_receive_message('POSITION_TARGET_LOCAL_NED')

            self.progress("Received local target: %s" % str(m))

        # Check the last received message
        if not (m.type_mask == (target_typemask | MAV_POS_TARGET_TYPE_MASK.LAST_BYTE) or m.type_mask == target_typemask):
            raise NotAchievedException("Did not receive proper mask: expected=%u or %u, got=%u" %
                                       ((target_typemask | MAV_POS_TARGET_TYPE_MASK.LAST_BYTE), target_typemask, m.type_mask))

        if vx - m.vx > 0.1:
            raise NotAchievedException("Did not receive proper target velocity vx: wanted=%f got=%f" % (vx, m.vx))

        if vy - m.vy > 0.1:
            raise NotAchievedException("Did not receive proper target velocity vy: wanted=%f got=%f" % (vy, m.vy))

        if vz_up - (-m.vz) > 0.1:
            raise NotAchievedException("Did not receive proper target velocity vz: wanted=%f got=%f" % (vz_up, -m.vz))

        self.progress("Received proper target velocity commands")

    def wait_for_local_velocity(self, vx, vy, vz_up, timeout=10):
        """ Wait for local target velocity"""

        # debug messages
        self.progress("Waiting for local NED velocity target: (%f, %f, %f)" % (vx, vy, -vz_up))
        self.progress("Setting LOCAL_POSITION_NED message rate to 10Hz")

        # set position local ned message stream rate
        self.set_message_rate_hz(mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 10)

        # wait for position local ned message
        tstart = self.get_sim_time()
        while self.get_sim_time_cached() - tstart < timeout:

            # get position target local ned message
            m = self.mav.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=1)

            # could not be able to get a valid target local ned message within given time
            if m is None:

                # raise an error that did not receive a valid target local ned message within given time
                raise NotAchievedException("Did not receive any position local ned message for 1 second!")

            # got a valid target local ned message within given time
            else:

                # debug message
                self.progress("Received local position ned message: %s" % str(m))

                # check if velocity values are in range
                if vx - m.vx <= 0.1 and vy - m.vy <= 0.1 and vz_up - (-m.vz) <= 0.1:

                    # get out of function
                    self.progress("Vehicle successfully reached to target velocity!")
                    return

        # raise an exception
        error_message = "Did not receive target velocities vx, vy, vz_up, wanted=(%f, %f, %f) got=(%f, %f, %f)"
        error_message = error_message % (vx, vy, vz_up, m.vx, m.vy, -m.vz)
        raise NotAchievedException(error_message)

    def earth_to_body(self, vector):
        r = mavextra.rotation(self.mav.messages["ATTITUDE"]).invert()
        #        print("r=%s" % str(r))
        return r * vector

    def precision_loiter_to_pos(self, x, y, z, send_bf=True, timeout=40):
        '''send landing_target messages at vehicle until it arrives at
        location to x, y, z from origin (in metres), z is *up*
        if send_bf is True then targets sent in body-frame, if False then sent in local FRD frame'''
        dest_ned = rotmat.Vector3(x, y, -z)
        tstart = self.get_sim_time()
        success_start = -1
        self.context_set_message_rate_hz('LOCAL_POSITION_NED', 10)
        while True:
            now = self.get_sim_time_cached()
            if now - tstart > timeout:
                raise NotAchievedException("Did not loiter to position!")
            m_pos = self.assert_receive_message('LOCAL_POSITION_NED')
            pos_ned = rotmat.Vector3(m_pos.x, m_pos.y, m_pos.z)
            #            print("dest_ned=%s" % str(dest_ned))
            #            print("pos_ned=%s" % str(pos_ned))
            delta_ef = dest_ned - pos_ned
            #            print("delta_ef=%s" % str(delta_ef))

            # determine if we've successfully navigated to close to
            # where we should be:
            dist = math.sqrt(delta_ef.x * delta_ef.x + delta_ef.y * delta_ef.y)
            dist_max = 1
            self.progress("dist=%f want <%f" % (dist, dist_max))
            if dist < dist_max:
                # success!  We've gotten within our target distance
                if success_start == -1:
                    success_start = now
                elif now - success_start > 10:
                    self.progress("Yay!")
                    break
            else:
                success_start = -1

            mav_frame = None
            if send_bf:
                # rotate delta_ef from NED to body frame FRD
                delta_bf = self.earth_to_body(delta_ef)
                print("delta_bf=%s" % str(delta_bf))
                angle_x = math.atan2(delta_bf.y, delta_bf.z)
                angle_y = -math.atan2(delta_bf.x, delta_bf.z)
                distance = math.sqrt(delta_bf.x * delta_bf.x +
                                     delta_bf.y * delta_bf.y +
                                     delta_bf.z * delta_bf.z)
                mav_frame = mavutil.mavlink.MAV_FRAME_BODY_FRD
            else:
                # rotate delta_ef from NED to Local FRD
                rotmat_yaw = Matrix3()
                rotmat_yaw.rotate_yaw(-self.mav.messages["ATTITUDE"].yaw) # negative yaw for earth->body
                delta_local_frd = rotmat_yaw * delta_ef
                angle_x = math.atan2(delta_local_frd.y, delta_local_frd.z)
                angle_y = -math.atan2(delta_local_frd.x, delta_local_frd.z)
                distance = math.sqrt(delta_local_frd.x * delta_local_frd.x +
                                     delta_local_frd.y * delta_local_frd.y +
                                     delta_local_frd.z * delta_local_frd.z)
                mav_frame = mavutil.mavlink.MAV_FRAME_LOCAL_FRD

            # att = self.mav.messages["ATTITUDE"]
            # print("r=%f p=%f y=%f" % (math.degrees(att.roll), math.degrees(att.pitch), math.degrees(att.yaw)))
            # print("angle_x=%s angle_y=%s" % (str(math.degrees(angle_x)), str(math.degrees(angle_y))))
            # print("distance=%s" % str(distance))

            self.mav.mav.landing_target_send(
                0, # time_usec
                1, # target_num
                mav_frame, # frame
                angle_x, # angle x (radians)
                angle_y, # angle y (radians)
                distance, # distance to target
                0.01, # size of target in radians, X-axis
                0.01 # size of target in radians, Y-axis
            )

    def set_servo_gripper_parameters(self):
        self.set_parameters({
            "GRIP_ENABLE": 1,
            "GRIP_TYPE": 1,
            "SIM_GRPS_ENABLE": 1,
            "SIM_GRPS_PIN": 8,
            "SERVO8_FUNCTION": 28,
        })

    def constrained_mount_pitch(self, pitch_angle_deg, mount_instance=1):
        PITCH_MIN = self.get_parameter("MNT%u_PITCH_MIN" % mount_instance, verbose=False)
        PITCH_MAX = self.get_parameter("MNT%u_PITCH_MAX" % mount_instance, verbose=False)
        return min(max(pitch_angle_deg, PITCH_MIN), PITCH_MAX)

    def test_mount_pitch(self, despitch, despitch_tolerance, mount_mode, timeout=10, hold=0, constrained=True):
        self.set_mount_mode(mount_mode)
        tstart = self.get_sim_time()
        success_start = 0

        while True:
            now = self.get_sim_time_cached()
            if now - tstart > timeout:
                raise NotAchievedException("Mount pitch not achieved")

            # We expect to achieve the desired pitch angle unless constrained by mount limits
            if constrained:
                despitch = self.constrained_mount_pitch(despitch)

            '''retrieve latest angles from GIMBAL_DEVICE_ATTITUDE_STATUS'''
            mount_roll, mount_pitch, mount_yaw, mount_yaw_is_absolute = self.get_mount_roll_pitch_yaw_deg()

            # self.progress("despitch=%f roll=%f pitch=%f yaw=%f" % (despitch, mount_roll, mount_pitch, mount_yaw))
            if abs(despitch - mount_pitch) > despitch_tolerance:
                self.progress("Mount pitch incorrect: got=%f want=%f (+/- %f)" %
                              (mount_pitch, despitch, despitch_tolerance))
                success_start = 0
                continue
            self.progress("Mount pitch correct: %f degrees == %f" %
                          (mount_pitch, despitch))
            if success_start == 0:
                success_start = now
            if now - success_start >= hold:
                self.progress("Mount pitch achieved")
                return

    # FIXME: change this to use wait_and_maintain
    def wait_mount_roll_pitch_yaw_deg(self, r=None, p=None, y=None, timeout=20, tolerance=5, minimum_duration=0):
        tstart = self.get_sim_time()
        settled_since = None
        while True:
            if self.get_sim_time_cached() - tstart > timeout:
                raise NotAchievedException("Did not get rpy")
            got_r, got_p, got_y, got_y_is_abs = self.get_mount_roll_pitch_yaw_deg()
            self.progress(f"want=({r}, {p}, {y})")
            self.progress(f"got=({got_r}, {got_p}, {got_y}, {got_y_is_abs})")
            passed = True
            for (want, got) in [(r, got_r), (p, got_p), (y, got_y)]:
                if want is None:
                    continue
                if abs(want - got) > tolerance:
                    passed = False
                    break
            if passed:
                if settled_since is None:
                    settled_since = self.get_sim_time_cached()
                if self.get_sim_time_cached() - settled_since >= minimum_duration:
                    self.progress("achieved mount rpy")
                    break
            else:
                settled_since = None

    def get_mount_roll_pitch_yaw_deg(self):
        '''return mount (aka gimbal) roll, pitch and yaw angles in degrees - in body frame'''
        # wait for gimbal attitude message
        m = self.assert_receive_message('GIMBAL_DEVICE_ATTITUDE_STATUS', timeout=5)
        return self.eulers_in_degrees_from_GIMBAL_DEVICE_ATTITUDE_STATUS(m)

    def eulers_in_degrees_from_GIMBAL_DEVICE_ATTITUDE_STATUS(self, m):
        # convert quaternion to euler angles and return

        yaw_is_absolute = m.flags & mavutil.mavlink.GIMBAL_DEVICE_FLAGS_YAW_LOCK
        q = quaternion.Quaternion(m.q)
        euler = q.euler
        return math.degrees(euler[0]), math.degrees(euler[1]), math.degrees(euler[2]), yaw_is_absolute

    def set_mount_mode(self, mount_mode):
        '''set mount mode'''
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_DO_MOUNT_CONFIGURE,
            p1=mount_mode,
            p2=0, # stabilize roll (unsupported)
            p3=0, # stabilize pitch (unsupported)
        )
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_MOUNT_CONFIGURE,
            p1=mount_mode,
            p2=0, # stabilize roll (unsupported)
            p3=0, # stabilize pitch (unsupported)
        )

    def test_mount_rc_targetting(self, pitch_rc_neutral=1500, do_rate_tests=True, pitch_tolerance=0.1):
        '''called in multipleplaces to make sure that mount RC targeting works

        pitch_tolerance defaults to the original tight 0.1deg check; backends whose
        actuator has a coarser confirmed physical resolution (e.g. a rate-only
        actuator closing an angle loop via a quantized speed command) may need to
        pass a wider value'''
        if True:
            self.context_push()
            self.set_parameters({
                'RC6_OPTION': 0,
                'RC11_OPTION': 212,    # MOUNT1_ROLL
                'RC12_OPTION': 213,    # MOUNT1_PITCH
                'RC13_OPTION': 214,    # MOUNT1_YAW
                'RC12_MIN': 1100,
                'RC12_MAX': 1900,
                'RC12_TRIM': 1500,
                'MNT1_PITCH_MIN': -45,
                'MNT1_PITCH_MAX': 45,
            })
            self.progress("Testing RC angular control")
            # default RC min=1100 max=1900
            self.set_rc_from_map({
                11: 1500,
                12: 1500,
                13: 1500,
            })
            self.test_mount_pitch(0, 1, mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING)
            self.progress("Testing RC input down 1/4 of its range in the output, should be down 1/4 range in output")
            rc12_in = 1400
            rc12_min = 1100 # default
            rc12_max = 1900 # default
            mpitch_min = -45.0
            mpitch_max = 45.0
            expected_pitch = (float(rc12_in-rc12_min)/float(rc12_max-rc12_min) * (mpitch_max-mpitch_min)) + mpitch_min
            self.progress("expected mount pitch: %f" % expected_pitch)
            if expected_pitch != -11.25:
                raise NotAchievedException("Calculation wrong - defaults changed?!")
            self.set_rc(12, rc12_in)
            self.test_mount_pitch(-11.25, pitch_tolerance, mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING)
            self.set_rc(12, 1800)
            self.test_mount_pitch(33.75, pitch_tolerance, mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING)
            self.set_rc_from_map({
                11: 1500,
                12: 1500,
                13: 1500,
            })

            try:
                self.context_push()
                self.set_parameters({
                    "RC12_MIN": 1000,
                    "RC12_MAX": 2000,
                    "MNT1_PITCH_MIN": -90,
                    "MNT1_PITCH_MAX": 10,
                })
                self.set_rc(12, 1000)
                self.test_mount_pitch(-90.00, pitch_tolerance, mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING)
                self.set_rc(12, 2000)
                self.test_mount_pitch(10.00, pitch_tolerance, mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING)
                self.set_rc(12, 1500)
                self.test_mount_pitch(-40.00, pitch_tolerance, mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING)
            finally:
                self.context_pop()

            self.set_rc(12, 1500)

            if do_rate_tests:
                self.test_mount_rc_targetting_rate_control()

            self.context_pop()

    def test_mount_rc_targetting_rate_control(self, pitch_rc_neutral=1500):
        if True:
            self.progress("Testing RC rate control")
            self.set_parameter('MNT1_RC_RATE', 10)
            self.test_mount_pitch(0, 1, mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING)
            # Note that we don't constrain the desired angle in the following so that we don't
            # timeout due to fetching Mount pitch limit params.
            self.set_rc(12, 1300)
            self.test_mount_pitch(-5, 1, mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING, constrained=False)
            self.test_mount_pitch(-10, 1, mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING, constrained=False)
            self.test_mount_pitch(-15, 1, mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING, constrained=False)
            self.test_mount_pitch(-20, 1, mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING, constrained=False)
            self.set_rc(12, 1700)
            self.test_mount_pitch(-15, 1, mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING, constrained=False)
            self.test_mount_pitch(-10, 1, mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING, constrained=False)
            self.test_mount_pitch(-5, 1, mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING, constrained=False)
            self.test_mount_pitch(0, 1, mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING, constrained=False)
            self.test_mount_pitch(5, 1, mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING, constrained=False)

            self.progress("Reverting to angle mode")
            self.set_parameter('MNT1_RC_RATE', 0)
            self.set_rc(12, 1500)
            self.test_mount_pitch(0, 0.1, mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING)

    def poll_camera_message(self, message_id, instance=1, **kwargs):
        '''request a cached native camera reply from the FC, retaining the camera identity'''
        return self.poll_message(
            message_id, response_source=(self.sysid_thismav(), mavutil.mavlink.MAV_COMP_ID_CAMERA + instance - 1), **kwargs)

    def wait_camera_initialised(self, instance, timeout=30):
        '''wait for the camera backend for instance (1-based) to report a
        vendor name in CAMERA_INFORMATION; it only does that once the camera
        has completed its handshake with the backend'''
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > timeout:
                raise NotAchievedException(
                    "Camera instance %u did not initialise" % instance)
            try:
                m = self.poll_camera_message('CAMERA_INFORMATION', instance=instance, timeout=5, p2=instance)
            except NotAchievedException:
                continue
            if bytes(m.vendor_name).split(b'\x00')[0]:
                return

    def camera_capture_statuses(self, count, timeout=10):
        '''request capture status and return image_status ordered by native camera component'''
        self.context_clear_collection('CAMERA_CAPTURE_STATUS')
        self.send_poll_message('CAMERA_CAPTURE_STATUS')
        tstart = self.get_sim_time()
        first_compid = mavutil.mavlink.MAV_COMP_ID_CAMERA
        while True:
            collection = {m.get_srcComponent(): m for m in self.context_collection('CAMERA_CAPTURE_STATUS')
                          if m.get_srcSystem() == self.sysid_thismav() and
                          first_compid <= m.get_srcComponent() < first_compid + count}
            if len(collection) == count:
                return [collection[compid].image_status for compid in sorted(collection)]
            if self.get_sim_time_cached() - tstart > timeout:
                raise NotAchievedException(
                    "Got %u CAMERA_CAPTURE_STATUS, wanted %u" %
                    (len(collection), count))
            self.mav.recv_match(type='CAMERA_CAPTURE_STATUS', blocking=True, timeout=0.1)

    def assert_mount_rpy(self, r, p, y, tolerance=1):
        '''assert mount atttiude in degrees'''
        got_r, got_p, got_y, yaw_is_absolute = self.get_mount_roll_pitch_yaw_deg()
        for (want, got, name) in (r, got_r, "roll"), (p, got_p, "pitch"), (y, got_y, "yaw"):
            if abs(want - got) > tolerance:
                raise NotAchievedException("%s incorrect; want=%f got=%f" %
                                           (name, want, got))

    def neutralise_gimbal(self):
        '''put mount into neutralise mode, assert it is at zero angles'''
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
            p7=mavutil.mavlink.MAV_MOUNT_MODE_NEUTRAL,
        )
        self.test_mount_pitch(0, 0, mavutil.mavlink.MAV_MOUNT_MODE_RETRACT)

    def hover_and_check_matched_frequency_with_fft_and_psd(self, dblevel=-15, minhz=200, maxhz=300, peakhz=None,
                                                           reverse=None, takeoff=True, instance=0, hover_time=15):
        '''Takeoff and hover, checking the noise against the provided db level and returning psd'''
        # find a motor peak
        if takeoff:
            self.takeoff(10, mode="ALT_HOLD")
            self.wait_altitude(8, 12, relative=True, minimum_duration=10)

        tstart, tend, hover_throttle = self.hover_for_interval(hover_time)
        self.do_RTL()

        psd = self.mavfft_fttd(1, instance, tstart * 1.0e6, tend * 1.0e6)

        # batch sampler defaults give 1024 fft and sample rate of 1kz so roughly 1hz/bin
        freq = psd["F"][numpy.argmax(psd["X"][minhz:maxhz]) + minhz] * (1000. / 1024.)
        peakdb = numpy.amax(psd["X"][minhz:maxhz])
        if peakdb < dblevel or (peakhz is not None and abs(freq - peakhz) / peakhz > 0.05):
            if reverse is not None:
                self.progress("Did not detect a motor peak, found %fHz at %fdB" % (freq, peakdb))
            else:
                raise NotAchievedException("Did not detect a motor peak, found %fHz at %fdB" % (freq, peakdb))
        else:
            if reverse is not None:
                raise NotAchievedException(
                    "Detected motor peak at %fHz, throttle %f%%, %fdB" %
                    (freq, hover_throttle, peakdb))
            else:
                self.progress("Detected motor peak at %fHz, throttle %f%%, %fdB" %
                              (freq, hover_throttle, peakdb))

        return freq, hover_throttle, peakdb, psd

    def hover_and_check_matched_frequency_with_fft(self, dblevel=-15, minhz=200, maxhz=300, peakhz=None,
                                                   reverse=None, takeoff=True, instance=0, hover_time=15):
        '''Takeoff and hover, checking the noise against the provided db level and returning peak db'''
        freq, hover_throttle, peakdb, psd = \
            self.hover_and_check_matched_frequency_with_fft_and_psd(dblevel, minhz,
                                                                    maxhz, peakhz, reverse, takeoff, instance, hover_time)
        return freq, hover_throttle, peakdb

    def hover_and_check_matched_frequency(self, *, dblevel=-15, minhz=200, maxhz=300, fftLength=32, peakhz=None):
        '''do a simple up-and-down test flight with current vehicle state.
        Check that the onboard filter comes up with the same peak-frequency that
        post-processing does.'''
        self.takeoff(10, mode="ALT_HOLD")
        tstart, tend, hover_throttle = self.hover_for_interval(15)
        self.do_RTL()

        psd = self.mavfft_fttd(1, 0, tstart * 1.0e6, tend * 1.0e6)

        # batch sampler defaults give 1024 fft and sample rate of 1kz so roughly 1hz/bin
        scale = 1000. / 1024.
        sminhz = int(minhz * scale)
        smaxhz = int(maxhz * scale)
        freq = psd["F"][numpy.argmax(psd["X"][sminhz:smaxhz]) + sminhz]
        peakdb = numpy.amax(psd["X"][sminhz:smaxhz])

        self.progress("Post-processing FFT detected motor peak at %fHz/%fdB, throttle %f%%" %
                      (freq, peakdb, hover_throttle))

        if peakdb < dblevel:
            raise NotAchievedException(
                "Detected motor peak not strong enough; want=%fdB got=%fdB" %
                (peakdb, dblevel))

        # caller can supply an expected frequency:
        if peakhz is not None and abs(freq - peakhz) / peakhz > 0.05:
            raise NotAchievedException(
                "Post-processing detected motor peak at wrong frequency; want=%fHz got=%fHz" %
                (peakhz, freq))

        # we have a peak make sure that the onboard filter detected
        # something close logging is at 10Hz

        # peak within resolution of FFT length
        pkAvg, nmessages = self.extract_median_FTN1_PkAvg_from_current_onboard_log(tstart, tend)
        self.progress("Onboard-FFT detected motor peak at %fHz (processed %d FTN1 messages)" % (pkAvg, nmessages))

        # accuracy is determined by sample rate and fft length, given
        # our use of quinn we could probably use half of this
        freqDelta = 1000. / fftLength
        if abs(pkAvg - freq) > freqDelta:
            raise NotAchievedException(
                "post-processed FFT does not  agree with onboard filter on peak frequency; onboard=%fHz post-processed=%fHz/%fdB" %  # noqa
                (pkAvg, freq, dblevel)
            )
        return freq

    def extract_median_FTN1_PkAvg_from_current_onboard_log(self, tstart, tend):
        '''extracts FTN1 messages from log, returns median of pkAvg values and
        the number of samples'''
        mlog = self.dfreader_for_current_onboard_log()
        freqs = []
        while True:
            m = mlog.recv_match(
                type='FTN1',
                blocking=False,
                condition="FTN1.TimeUS>%u and FTN1.TimeUS<%u" % (tstart * 1.0e6, tend * 1.0e6))
            if m is None:
                break
            freqs.append(m.PkAvg)
        return numpy.median(numpy.asarray(freqs)), len(freqs)

    def test_gyro_fft_harmonic(self, averaging):
        """Use dynamic harmonic notch to control motor noise with harmonic matching of the first harmonic."""
        # basic gyro sample rate test
        self.progress("Flying with gyro FFT harmonic - Gyro sample rate")
        self.start_subtest("Hover to calculate approximate hover frequency")
        # magic tridge EKF type that dramatically speeds up the test
        self.set_parameters({
            "AHRS_EKF_TYPE": 10,
            "EK2_ENABLE": 0,
            "EK3_ENABLE": 0,
            "INS_LOG_BAT_MASK": 3,
            "INS_LOG_BAT_OPT": 0,
            "INS_GYRO_FILTER": 100,
            "INS_FAST_SAMPLE": 0,
            "LOG_BITMASK": 958,
            "LOG_DISARMED": 0,
            "SIM_DRIFT_SPEED": 0,
            "SIM_DRIFT_TIME": 0,
            "FFT_THR_REF": self.get_parameter("MOT_THST_HOVER"),
            "SIM_GYR1_RND": 20,  # enable a noisy gyro
        })

        # motor peak enabling FFT will also enable the arming
        # check, self-testing the functionality
        self.set_parameters({
            "FFT_ENABLE": 1,
            "FFT_MINHZ": 50,
            "FFT_MAXHZ": 450,
            "FFT_SNR_REF": 10,
        })
        if averaging:
            self.set_parameter("FFT_NUM_FRAMES", 8)

        # Step 1: inject actual motor noise and use the FFT to track it
        self.set_parameters({
            "SIM_VIB_MOT_MAX": 250, # gives a motor peak at about 175Hz
            "FFT_WINDOW_SIZE": 64,
            "FFT_WINDOW_OLAP": 0.75,
        })

        self.reboot_sitl()
        freq = self.hover_and_check_matched_frequency(
            dblevel=-15,
            minhz=100,
            maxhz=250,
            fftLength=64,
        )

        # Step 2: add a second harmonic and check the first is still tracked
        self.start_subtest("Add a fixed frequency harmonic at twice the hover frequency "
                           "and check the right harmonic is found")
        self.set_parameters({
            "SIM_VIB_FREQ_X": freq * 2,
            "SIM_VIB_FREQ_Y": freq * 2,
            "SIM_VIB_FREQ_Z": freq * 2,
            "SIM_VIB_MOT_MULT": 0.25,  # halve the motor noise so that the higher harmonic dominates
        })
        self.reboot_sitl()

        self.hover_and_check_matched_frequency(
            dblevel=-15,
            minhz=100,
            maxhz=250,
            fftLength=64,
            peakhz=None,
        )

        # Step 3: switch harmonics mid flight and check for tracking
        self.start_subtest("Switch harmonics mid flight and check the right harmonic is found")
        self.set_parameter("FFT_HMNC_PEAK", 0)
        self.reboot_sitl()

        self.takeoff(10, mode="ALT_HOLD")

        hover_time = 10
        tstart, tend_unused, hover_throttle = self.hover_for_interval(hover_time)

        self.progress("Switching motor vibration multiplier")
        self.set_parameter("SIM_VIB_MOT_MULT", 5.0)

        tstart_unused, tend, hover_throttle = self.hover_for_interval(hover_time)

        self.do_RTL()

        # peak within resolution of FFT length, the highest energy peak switched but our detection should not
        pkAvg, nmessages = self.extract_median_FTN1_PkAvg_from_current_onboard_log(tstart, tend)

        freqDelta = 1000. / self.get_parameter("FFT_WINDOW_SIZE")

        if abs(pkAvg - freq) > freqDelta:
            raise NotAchievedException("FFT did not detect a harmonic motor peak, found %f, wanted %f" % (pkAvg, freq))

        # Step 4: dynamic harmonic
        self.start_subtest("Enable dynamic harmonics and make sure both frequency peaks are attenuated")
        # find a motor peak
        freq, hover_throttle, peakdb = self.hover_and_check_matched_frequency_with_fft(-15, 100, 350)

        # now add a dynamic notch and check that the peak is squashed
        self.set_parameters({
            "INS_LOG_BAT_OPT": 2,
            "INS_HNTCH_ENABLE": 1,
            "INS_HNTCH_HMNCS": 1,
            "INS_HNTCH_MODE": 4,
            "INS_HNTCH_FREQ": freq,
            "INS_HNTCH_REF": hover_throttle/100.0,
            "INS_HNTCH_ATT": 100,
            "INS_HNTCH_BW": freq/2,
            "INS_HNTCH_OPTS": 3,
        })
        self.reboot_sitl()

        # 5db is far in excess of the attenuation that the double dynamic-harmonic notch is able
        # to provide (-7dB on average), but without the notch the peak is around 20dB so still a safe test
        self.hover_and_check_matched_frequency_with_fft(5, 100, 350, reverse=True)

        self.set_parameters({
            "SIM_VIB_FREQ_X": 0,
            "SIM_VIB_FREQ_Y": 0,
            "SIM_VIB_FREQ_Z": 0,
            "SIM_VIB_MOT_MULT": 1.0,
        })
        # prevent update parameters from messing with the settings when we pop the context
        self.set_parameter("FFT_ENABLE", 0)

    # every parameter the harmonic notch has
    # (libraries/Filter/HarmonicNotchFilter.cpp), which is what the FFT
    # notch tune writes and saves for itself
    harmonic_notch_params = [
        "INS_HNTCH_%s" % suffix
        for suffix in ("ENABLE", "FREQ", "BW", "ATT", "HMNCS", "REF",
                       "MODE", "OPTS", "FM_RAT")
    ]

    def fly_guided_move_to(self, destination: Location, timeout=30):
        '''move to a (frame-aware) Location; the destination is sent as an
        absolute (AMSL) altitude'''
        destination = self.change_alt_frame(destination, AltFrame.ABSOLUTE)
        tstart = self.get_sim_time()
        self.mav.mav.set_position_target_global_int_send(
            0, # timestamp
            1, # target system_id
            1, # target component id
            mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
            MAV_POS_TARGET_TYPE_MASK.POS_ONLY | MAV_POS_TARGET_TYPE_MASK.LAST_BYTE, # mask specifying use-only-lat-lon-alt
            int(destination.lat * 1e7), # lat
            int(destination.lng * 1e7), # lon
            destination.get_alt_m(AltFrame.ABSOLUTE), # alt
            0, # vx
            0, # vy
            0, # vz
            0, # afx
            0, # afy
            0, # afz
            0, # yaw
            0, # yawrate
        )
        while True:
            if self.get_sim_time() - tstart > timeout:
                raise NotAchievedException()
            delta = self.get_distance(self.get_location(), destination)
            self.progress("delta=%f (want <1)" % delta)
            if delta < 1:
                break
        # the vehicle crosses the 1m threshold while still travelling at
        # a couple of metres/second, and callers go on to land or to
        # measure where it ended up; wait for it to settle on the
        # destination rather than catching it mid-approach
        self.wait_groundspeed(0, 0.5, timeout=30)

    def loiter_requires_position(self):
        # ensure we can't switch to LOITER without position
        self.progress("Ensure we can't enter LOITER without position")
        self.context_push()
        self.set_parameters({
            "GPS1_TYPE": 2,
            "SIM_GPS1_ENABLE": 0,
        })
        # if there is no GPS at all then we must direct EK3 to not use
        # it at all.  Otherwise it will never initialise, as it wants
        # to calculate the lag and size its delay buffers accordingly.
        self.set_parameters({
            "EK3_SRC1_POSXY": 0,
            "EK3_SRC1_VELZ": 0,
            "EK3_SRC1_VELXY": 0,
        })
        self.reboot_sitl()
        # allow the vehicle to settle after reboot; the EKF flags come
        # good long before the accelerometers are consistent enough to
        # pass the arm-time "Accels inconsistent" check
        self.delay_sim_time(30, reason="accels/gyros to settle")

        # check for expected EKF flags
        ahrs_ekf_type = self.get_parameter("AHRS_EKF_TYPE")
        expected_ekf_flags = (mavutil.mavlink.ESTIMATOR_ATTITUDE |
                              mavutil.mavlink.ESTIMATOR_VELOCITY_VERT |
                              mavutil.mavlink.ESTIMATOR_POS_VERT_ABS |
                              mavutil.mavlink.ESTIMATOR_CONST_POS_MODE)
        if ahrs_ekf_type == 2:
            expected_ekf_flags = expected_ekf_flags | mavutil.mavlink.ESTIMATOR_PRED_POS_HORIZ_REL
        self.wait_ekf_flags(expected_ekf_flags, 0, timeout=120)

        # arm in Stabilize and attempt to switch to Loiter
        self.change_mode('STABILIZE')
        self.arm_vehicle()
        self.context_collect('STATUSTEXT')
        self.run_cmd_do_set_mode(
            "LOITER",
            want_result=mavutil.mavlink.MAV_RESULT_FAILED)
        self.wait_statustext("requires position", check_context=True)
        self.disarm_vehicle()
        self.context_pop()
        self.reboot_sitl()

    def ArmFeatures(self):
        '''Arm features'''
        self.loiter_requires_position()

        super(AutoTestCopter, self).ArmFeatures()

    def initial_mode(self):
        return "STABILIZE"

    def initial_mode_switch_mode(self):
        return "STABILIZE"

    def default_mode(self):
        return "STABILIZE"

    def rc_defaults(self):
        ret = super(AutoTestCopter, self).rc_defaults()
        ret[3] = 1000
        ret[5] = 1800 # mode switch
        return ret

    def check_avoidance_corners(self):
        self.takeoff(10, mode="LOITER")
        here = self.get_location()
        here_alt_amsl = here.get_alt_m(AltFrame.ABSOLUTE)
        # the vehicle starts a test at home but with whatever heading
        # the previous test left it; face west like the other corner
        # legs explicitly face their travel direction:
        self.reach_heading_manual(270)
        self.set_rc(2, 1400)
        west_loc = Location(-35.363007, 149.164911, here_alt_amsl, AltFrame.ABSOLUTE)
        self.wait_location(west_loc, accuracy=6)
        north_loc = Location(-35.362908, 149.165051, here_alt_amsl, AltFrame.ABSOLUTE)
        self.reach_heading_manual(0)
        self.wait_location(north_loc, accuracy=6, timeout=200)
        self.reach_heading_manual(90)
        east_loc = Location(-35.363013, 149.165194, here_alt_amsl, AltFrame.ABSOLUTE)
        self.wait_location(east_loc, accuracy=6)
        self.reach_heading_manual(225)
        self.wait_location(west_loc, accuracy=6, timeout=200)
        self.set_rc(2, 1500)
        self.do_RTL()

    def get_global_position_int(self, timeout=30):
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > timeout:
                raise NotAchievedException("Did not get good global_position_int")
            m = self.assert_receive_message('GLOBAL_POSITION_INT')
            self.progress("GPI: %s" % str(m))
            if m is None:
                continue
            if m.lat != 0 or m.lon != 0:
                return m

    def wait_generator_speed_and_state(self, rpm_min, rpm_max, want_state, timeout=240):
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > timeout:
                raise NotAchievedException("Did not move to state/speed")

            m = self.assert_receive_message("GENERATOR_STATUS", timeout=10)

            if m.generator_speed < rpm_min:
                self.progress("Too slow (%u<%u)" % (m.generator_speed, rpm_min))
                continue
            if m.generator_speed > rpm_max:
                self.progress("Too fast (%u>%u)" % (m.generator_speed, rpm_max))
                continue
            if m.status != want_state:
                self.progress("Wrong state (got=%u want=%u)" % (m.status, want_state))
            break
        self.progress("Got generator speed and state")

    def wait_efi_fuel_consumed(self, min_fuel_consumed, timeout=60):
        '''wait until EFI_STATUS reports at least min_fuel_consumed'''
        self.progress("Waiting for %f fuel to have been consumed" % min_fuel_consumed)
        tstart = self.get_sim_time()
        while True:
            m = self.assert_receive_message('EFI_STATUS', verbose=True)
            if m.fuel_consumed >= min_fuel_consumed:
                return m
            if self.get_sim_time_cached() - tstart > timeout:
                raise NotAchievedException(
                    "Insufficient fuel consumed (want>=%f got=%f)" %
                    (min_fuel_consumed, m.fuel_consumed))

    def fly_rangefinder_drivers_fly(self, rangefinders):
        '''ensure rangefinder gives height-above-ground'''
        expected_alt = 5
        self.takeoff(expected_alt, mode='GUIDED', altitude_max=expected_alt+5, alt_minimum_duration=5)
        ds = self.assert_receive_message("DISTANCE_SENSOR")
        gpi = self.assert_receive_message('GLOBAL_POSITION_INT')
        delta = abs(ds.current_distance*0.01 - gpi.relative_alt/1000.0)
        alt_msg = f"{ds.current_distance*0.01=} disagrees with global-position-int.relative_alt ({gpi.relative_alt/1000.0}) by {delta}m"  # NOQA:E501
        self.progress(alt_msg)
        self.progress(f"Terrain report: {self.mav.messages['TERRAIN_REPORT']}")
        if delta > 1:
            raise NotAchievedException(alt_msg)

        for i in range(0, len(rangefinders)):
            name = rangefinders[i]
            self.progress("i=%u (%s)" % (i, name))
            ds = self.assert_receive_message(
                "DISTANCE_SENSOR",
                timeout=2,
                condition=f"DISTANCE_SENSOR.id=={i}",
                verbose=True,
            )
            if abs(ds.current_distance/100.0 - gpi.relative_alt/1000.0) > 1:
                raise NotAchievedException(
                    "distance sensor.current_distance (%f) (%s) disagrees with global-position-int.relative_alt (%s)" %
                    (ds.current_distance/100.0, name, gpi.relative_alt/1000.0))

        self.land_and_disarm()

        self.progress("Ensure RFND messages in log")
        if not self.current_onboard_log_contains_message("RFND"):
            raise NotAchievedException("No RFND messages in log")

    def RangeFinderDriversMaxAlt_FlyDriver(self,
                                           name : str,
                                           rngfnd_type : int,
                                           simname : str,
                                           maxalt : float,
                                           sqalt : float | None = None,
                                           sq_at_sqalt : float | None = None,
                                           expected_statustext : str | None = None,
                                           ):
        '''test max-height behaviour'''
        # lightwareserial goes to 130m when out of range
        self.progress(f"Flying {name}")
        self.set_parameters({
            "SERIAL4_PROTOCOL": 9,
            "RNGFND1_TYPE": rngfnd_type,
            "WP_SPD_UP": 10,  # m/s
        })
        self.customise_SITL_commandline([
            f"--serial4=sim:{simname}",
        ])
        takeoff_alt = sqalt
        if sqalt is None:
            takeoff_alt = maxalt
        self.context_set_message_rate_hz('RANGEFINDER', self.sitl_streamrate())
        self.takeoff(takeoff_alt, mode='GUIDED', timeout=240, altitude_max=takeoff_alt+0.5)
        self.assert_rangefinder_distance_between(takeoff_alt-5, takeoff_alt+5)

        self.wait_rangefinder_distance(takeoff_alt-5, takeoff_alt+5)

        rf_bit = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_LASER_POSITION

        self.assert_sensor_state(rf_bit, present=True, enabled=True, healthy=True)
        if sq_at_sqalt is not None:
            self.assert_distance_sensor_quality(sq_at_sqalt)

        self.progress("Moving higher to be out of max rangefinder range")
        self.fly_guided_move_local(0, 0, takeoff_alt + 50)

        # sensor remains healthy even out-of-range
        self.assert_sensor_state(rf_bit, present=True, enabled=True, healthy=True)

        self.assert_distance_sensor_quality(1)

        self.progress("Test behaviour above 655.35 metres (UINT16_MAX cm)")
        self.context_push()
        self.context_collect('STATUSTEXT')
#        self.send_debug_trap()
        self.fly_guided_move_local(0, 0, 700)
        self.assert_sensor_state(rf_bit, present=True, enabled=True, healthy=True)
        self.assert_distance_sensor_quality(1)
        if expected_statustext is not None:
            self.wait_statustext(expected_statustext, check_context=True)
        self.context_pop()

        # life's too short for soft landings:
        self.disarm_vehicle(force=True)

    def test_replay_wind_and_airspeed_bit(self):
        self.set_parameters({
            "LOG_REPLAY": 1,
            "LOG_DISARMED": 1,
            "EK3_ENABLE": 1,
            "WP_YAW_BEHAVIOR": 1, # face into wind

            "EK3_DRAG_BCOEF_X": 17.209, # wind estimation must be on for airspeed
            "EK3_DRAG_BCOEF_Y": 17.209, # these are the suggested values printed out by sitl
            "EK3_DRAG_MCOEF": 0.209,
            "SIM_WIND_SPD": 3,

            "ARSPD_ENABLE": 1,
            "ARSPD_TYPE": 100,
            "ARSPD_USE": 1, # technically not actually supported on copter
        })
        self.reboot_sitl()

        self.wait_sensor_state(mavutil.mavlink.MAV_SYS_STATUS_LOGGING, True, True, True)

        current_log_filepath = self.current_onboard_log_filepath()
        self.progress("Current log path: %s" % str(current_log_filepath))

        self.change_mode("LOITER")
        self.wait_ready_to_arm(require_absolute=True)
        # make sure airspeed is being used
        self.wait_sensor_state(mavutil.mavlink.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE, True, True, True)
        self.arm_vehicle()
        self.takeoffAndMoveAway()

        # make sure airspeed was fused at some point after some flying
        m = self.assert_receive_message('EKF_STATUS_REPORT')
        if m.airspeed_variance == 0:
            raise NotAchievedException("never fused airspeed")

        self.do_RTL()

        self.reboot_sitl()

        return current_log_filepath

    def test_replay_bit(self, bit):

        self.context_push()
        current_log_filepath = bit()

        self.progress("Running replay on (%s) (%u bytes)" % (
            (current_log_filepath, os.path.getsize(current_log_filepath))
        ))

        # a log with dropped blocks is incomplete and cannot be replayed
        # faithfully, so check before wasting time on the replay itself:
        self.assert_log_has_no_dropped_blocks(current_log_filepath)

        self.zero_throttle()
        self.run_replay(current_log_filepath)

        replay_log_filepath = self.current_onboard_log_filepath()

        self.context_pop()

        self.progress("Replay log path: %s" % str(replay_log_filepath))

        check_replay = util.load_local_module("Tools/Replay/check_replay.py")

        ok = check_replay.check_log(replay_log_filepath, self.progress, verbose=True)
        if not ok:
            raise NotAchievedException("check_replay (%s) failed" % current_log_filepath)

    def get_ground_effect_duration_from_current_onboard_log(self, bit, ignore_multi=False):
        '''returns a duration in seconds we were expecting to interact with
        the ground.  Will die if there's more than one such block of
        time and ignore_multi is not set (will return first duration
        otherwise)
        '''
        ret = []
        dfreader = self.dfreader_for_current_onboard_log()
        seen_expected_start_TimeUS = None
        first = None
        last = None
        while True:
            m = dfreader.recv_match(type="XKF4")
            if m is None:
                break
            last = m
            if first is None:
                first = m
            # self.progress("%s" % str(m))
            expected = m.SS & (1 << bit)
            if expected:
                if seen_expected_start_TimeUS is None:
                    seen_expected_start_TimeUS = m.TimeUS
                    continue
            else:
                if seen_expected_start_TimeUS is not None:
                    duration = (m.TimeUS - seen_expected_start_TimeUS)/1000000.0
                    ret.append(duration)
                    seen_expected_start_TimeUS = None
        if seen_expected_start_TimeUS is not None:
            duration = (last.TimeUS - seen_expected_start_TimeUS)/1000000.0
            ret.append(duration)
        return ret

    def get_takeoffexpected_durations_from_current_onboard_log(self, ignore_multi=False):
        return self.get_ground_effect_duration_from_current_onboard_log(11, ignore_multi=ignore_multi)

    def get_touchdownexpected_durations_from_current_onboard_log(self, ignore_multi=False):
        return self.get_ground_effect_duration_from_current_onboard_log(12, ignore_multi=ignore_multi)

    def _MAV_CMD_CONDITION_YAW(self, command):
        self.start_subtest("absolute")
        self.takeoff(20, mode='GUIDED')

        m = self.assert_receive_message('VFR_HUD')
        initial_heading = m.heading

        self.progress("Ensuring initial heading is steady")
        target = initial_heading
        command(
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            p1=target,  # target angle
            p2=10,  # degrees/second
            p3=1,  # -1 is counter-clockwise, 1 clockwise
            p4=0,  # 1 for relative, 0 for absolute
        )
        self.wait_heading(target, minimum_duration=2, timeout=50)
        self.wait_yaw_speed(0)

        degsecond = 2

        def rate_watcher(mav, m):
            if m.get_type() != 'ATTITUDE':
                return
            if abs(math.degrees(m.yawspeed)) > 5*degsecond:
                raise NotAchievedException("Moved too fast (%f>%f)" %
                                           (math.degrees(m.yawspeed), 5*degsecond))
        self.install_message_hook_context(rate_watcher)
        self.progress("Yaw CW 60 degrees")
        target = initial_heading + 60
        part_way_target = initial_heading + 10
        command(
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            p1=target,     # target angle
            p2=degsecond,  # degrees/second
            p3=1,          # -1 is counter-clockwise, 1 clockwise
            p4=0,          # 1 for relative, 0 for absolute
        )
        self.wait_heading(part_way_target)
        self.wait_heading(target, minimum_duration=2)

        self.progress("Yaw CCW 60 degrees")
        target = initial_heading
        part_way_target = initial_heading + 30
        command(
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            p1=target,  # target angle
            p2=degsecond,  # degrees/second
            p3=-1,  # -1 is counter-clockwise, 1 clockwise
            p4=0,  # 1 for relative, 0 for absolute
        )
        self.wait_heading(part_way_target)
        self.wait_heading(target, minimum_duration=2)

        self.disarm_vehicle(force=True)
        self.reboot_sitl()

    def tests1a(self):
        '''return list of all tests'''
        ret = super(AutoTestCopter, self).tests()  # about 5 mins and ~20 initial tests from autotest/vehicle_test_suite.py
        ret.extend([
            self.SetpointGlobalVel,
            self.SetpointBadVel,
            self.FenceRelativeToHomeMaxAltOriginAbove,
            self.FenceRelativeToHomeCliff,
        ])
        ret.extend(self.discovered_tests("CopterTests1a"))
        return ret

    def tests1b(self):
        '''return list of all tests'''
        ret = []
        ret.extend(self.discovered_tests("CopterTests1b"))
        return ret

    def tests1c(self):
        '''return list of all tests'''
        ret = ([
            self.FenceRelativeToHomeMaxAlt,
            self.SixCompassCalibrationAndReordering,
        ])
        ret.extend(self.discovered_tests("CopterTests1c"))
        return ret

    def tests1d(self):
        '''return list of all tests'''
        ret = ([
            self.SetpointGlobalPos,
            self.FenceRelativePreArms,
            self.IBus,
        ])
        ret.extend(self.discovered_tests("CopterTests1d"))
        return ret

    def tests1e(self):
        '''return list of all tests'''
        ret = ([
            self.FenceRelativeToHomeMinAltOriginAbove,
            self.FenceRelativeToOriginMaxAltHomeAbove,
            self.FenceRelativeToTerrainMaxAlt,
            self.FenceRelativeToTerrainMinAlt,
            self.DataFlashErase,
        ])
        ret.extend(self.discovered_tests("CopterTests1e"))
        return ret

    def tests2a(self):
        '''return list of all tests'''
        ret = ([
            # something about SITLCompassCalibration appears to fail
            # this one, so we put it first:
            self.FixedYawCalibration,
            # we run this single 8min-and-40s test on its own, apart
            #   from requiring FixedYawCalibration right before it
            #   because without it, it fails to calibrate this
            #   autotest appears to interfere with
            #   FixedYawCalibration, no idea why.
            self.SITLCompassCalibration,
            self.FenceRelativeToOriginMinAltHomeAbove,
            self.FenceRelativeToAMSLMaxAlt,
            self.MotorTest,
            self.DataFlash,
        ])
        ret.extend(self.discovered_tests("CopterTests2a"))
        return ret

    def test_rplidar(self, sim_device_name, expected_distances):
        '''plonks a Copter with a RPLidarA2 in the middle of a simulated field
        of posts and checks that the measurements are what we expect.'''
        self.set_parameters({
            "SERIAL5_PROTOCOL": 11,
            "PRX1_TYPE": 5,
        })
        self.customise_SITL_commandline([
            "--serial5=sim:%s:" % sim_device_name,
            "--home", "51.8752066,14.6487840,0,0",  # SITL has "posts" here
        ])

        self.wait_ready_to_arm()

        wanting_distances = copy.copy(expected_distances)
        tstart = self.get_sim_time()
        timeout = 60
        while True:
            now = self.get_sim_time_cached()
            if now - tstart > timeout:
                raise NotAchievedException("Did not get all distances")
            m = self.mav.recv_match(type="DISTANCE_SENSOR",
                                    blocking=True,
                                    timeout=1)
            if m is None:
                continue
            self.progress("Got (%s)" % str(m))
            if m.orientation not in wanting_distances:
                continue
            if abs(m.current_distance - wanting_distances[m.orientation]) > 5:
                self.progress("Wrong distance orient=%u want=%u got=%u" %
                              (m.orientation,
                               wanting_distances[m.orientation],
                               m.current_distance))
                continue
            self.progress("Correct distance for orient %u (want=%u got=%u)" %
                          (m.orientation,
                           wanting_distances[m.orientation],
                           m.current_distance))
            del wanting_distances[m.orientation]
            if len(wanting_distances.items()) == 0:
                break

    def _MAV_CMD_DO_FLIGHTTERMINATION(self, command):
        self.set_parameters({
            "MAV_GCS_SYSID": self.mav.source_system,
            "DISARM_DELAY": 0,
        })
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.context_collect('STATUSTEXT')
        command(mavutil.mavlink.MAV_CMD_DO_FLIGHTTERMINATION, p1=1)
        self.wait_disarmed()
        self.reboot_sitl()

    def start_flying_simple_relhome_mission(self, items):
        '''uploads items, changes mode to auto, waits ready to arm and arms
        vehicle.  If the first item it a takeoff you can expect the
        vehicle to fly after this method returns
        '''

        self.upload_simple_relhome_mission(items)

        self.set_parameter("AUTO_OPTIONS", 3)
        self.change_mode('AUTO')
        self.wait_ready_to_arm()

        self.arm_vehicle()

    def learn_compass_offsets_in_flight(self):
        '''take off and fly so the EKF learns a new compass X offset, returns the offset it should learn'''
        self.takeoff(30, mode='ALT_HOLD')
        # prevent EKF switching to good compass:
        self.set_parameters({
            'COMPASS_USE2': 0,
            'COMPASS_USE3': 0,
        })
        self.assert_parameter_value("COMPASS_OFS_X", 20, epsilon=30)
        # the firmware is about to learn and save these, so set them to
        # the values they already have; that way the suite knows what to
        # restore them to at context pop time.  set_and_save_offsets()
        # writes all three axes, not just the one we assert on:
        self.set_parameters(self.get_parameters([
            "COMPASS_OFS_X", "COMPASS_OFS_Y", "COMPASS_OFS_Z",
        ]))
        new_compass_ofs_x = 200
        self.set_parameters({
            "SIM_MAG1_OFS_X": new_compass_ofs_x,
        })
        self.set_parameter("COMPASS_LEARN", 2)  # 2 is Copy-from-EKF

        self.fly_compass_learning_manoeuvres()
        return new_compass_ofs_x

    def fly_compass_learning_manoeuvres(self):
        '''commence silly flying to give the EKF as much information as possible to converge its compass offset estimates'''
        self.set_rc_from_map({4: 1450, 1: 1450})
        for i in range(0, 5):  # we descend through all of this:
            self.change_mode('LOITER')
            self.delay_sim_time(10, reason="compass learn data to accumulate")
            self.change_mode('ALT_HOLD')
            self.change_mode('FLIP')

        self.set_parameter('ATC_ANGLE_MAX', 70)
        self.change_mode('ALT_HOLD')
        for j in 1000, 2000:
            for i in 1, 2, 4:
                self.set_rc(i, j)
                self.delay_sim_time(10, reason="compass learn data to accumulate")
        self.set_rc_from_map({1: 1500, 2: 1500, 4: 1500})

    def fly_mission_NAV_LOITER_TURNS(self, ccw=False, circle_centre_loc=None, radius_m=30):
        '''fly a mission containing a single LOITER_TURNS item, checking the
        circle is flown in the direction the mission item asks for.  A
        negative loiter radius asks for a counter-clockwise circle.  If
        circle_centre_loc is None then no location is supplied in the
        mission item, so the vehicle circles where it is when the item
        starts - which is over home'''
        lat = 0
        lng = 0
        if circle_centre_loc is None:
            circle_centre_loc = self.home_position_as_location()
        else:
            lat = int(circle_centre_loc.lat*1e7)
            lng = int(circle_centre_loc.lng*1e7)
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),
            self.create_MISSION_ITEM_INT(
                mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS,
                p1=1,
                p3=-radius_m if ccw else radius_m,
                x=lat,
                y=lng,
                z=30,  # circle is 10m higher than takeoff
                frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            ),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.change_mode('AUTO')
        self.set_parameter('AUTO_OPTIONS', 3)
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.wait_current_waypoint(2)
        self.wait_circle_direction(circle_centre_loc, radius_m, ccw)
        self.wait_disarmed()

    def do_land(self):
        self.change_mode('LAND')
        self.wait_disarmed()

    def FenceRelative_TakeoffMode(self):
        '''method for the FenceRelative test to call'''
        return 'LOITER'

    def FenceRelative_params(self):
        ret = super().FenceRelative_params()
        ret["AVOID_ENABLE"] = 0
        return ret

    def tests2b(self):
        '''return list of all tests'''
        ret = ([
            self.Gripper,
            self.Button,
            self.FenceRelativeToHomeMinAlt,
            self.FenceRelativeToAMSLMinAlt,
            self.MSPVTXConfig,
            self.MSPDisplayPortVTXConfig,
            self.GPSTypes,
            self.MultipleGPS,
        ])
        ret.extend(self.discovered_tests("CopterTests2b"))
        return ret

    def tests2c(self):
        '''return list of all tests'''
        ret = ([
            self.FenceRelativeToOriginMaxAlt,
            self.FenceRelativeToOriginMinAlt,
            self.CRSF,
            self.SerialPassthroughRCIN,
            self.CRSFParameterMenu,
            self.CRSFParameterMenuScripted,
            self.CRSFParameterMenuUnusedSlot,
            self.CRSFParameterMenuChunks,
            self.CRSFParameterMenuLongCommandReply,
            self.CRSFParameterMenuFolderLimits,
            self.NoArmWithoutMissionItems,
        ])
        ret.extend(self.discovered_tests("CopterTests2c"))
        return ret

    def tests2d(self):
        '''return list of all tests'''
        ret = ([
            self.TestLocalHomePosition,
            self.FenceRelativeToAMSLCliff,
            self.CompassReordering,
            self.LoggingFormat,
        ])
        ret.extend(self.discovered_tests("CopterTests2d"))
        return ret

    def testcan(self):
        ret = ([
            self.TestLogDownloadMAVProxyCAN,
        ])
        ret.extend(self.discovered_tests("CAN"))
        return ret

    def testcanbatt(self):
        ret = []
        ret.extend(self.discovered_tests("BattCAN"))
        return ret

    def tests(self):
        ret = []
        ret.extend(self.tests1a())
        ret.extend(self.tests1b())
        ret.extend(self.tests1c())
        ret.extend(self.tests1d())
        ret.extend(self.tests1e())
        ret.extend(self.tests2a())
        ret.extend(self.tests2b())
        ret.extend(self.tests2c())
        ret.extend(self.tests2d())
        return ret


class AutoTestCopterTests1a(AutoTestCopter):
    def tests(self):
        return self.tests1a()


class AutoTestCopterTests1b(AutoTestCopter):
    def tests(self):
        return self.tests1b()


class AutoTestCopterTests1c(AutoTestCopter):
    def tests(self):
        return self.tests1c()


class AutoTestCopterTests1d(AutoTestCopter):
    def tests(self):
        return self.tests1d()


class AutoTestCopterTests1e(AutoTestCopter):
    def tests(self):
        return self.tests1e()


class AutoTestCopterTests2a(AutoTestCopter):
    def tests(self):
        return self.tests2a()


class AutoTestCopterTests2b(AutoTestCopter):
    def tests(self):
        return self.tests2b()


class AutoTestCopterTests2c(AutoTestCopter):
    def tests(self):
        return self.tests2c()


class AutoTestCopterTests2d(AutoTestCopter):
    def tests(self):
        return self.tests2d()


class AutoTestCAN(AutoTestCopter):

    def tests(self):
        return self.testcan()


class AutoTestBattCAN(AutoTestCopter):

    def tests(self):
        return self.testcanbatt()
