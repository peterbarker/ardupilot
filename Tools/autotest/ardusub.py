'''
Dive ArduSub in SITL

Depth of water is 50m, the ground is flat
Parameters are in-code defaults plus default_params/sub.parm

AP_FLAKE8_CLEAN
'''

import os
import re


from pymavlink import mavutil

import vehicle_test_suite

from vehicle_test_suite import AltFrame
from vehicle_test_suite import Location
from vehicle_test_suite import NotAchievedException

# get location of scripts
testdir = os.path.dirname(os.path.realpath(__file__))

SITL_START_LOCATION = Location(33.810313, -118.393867, 0, AltFrame.ABSOLUTE)
SITL_START_HEADING = 185

# Extract true range from STATUSTEXT messages sent by sub_test_synthetic_seafloor.lua
RE_TR_SEARCH = re.compile(r'#TR#\s*([-+]?\d*\.?\d+)')


class Joystick():
    Pitch = 1
    Roll = 2
    Throttle = 3
    Yaw = 4
    Forward = 5
    Lateral = 6


# Values for EK3_MAG_CAL
class MagCal():
    WHEN_FLYING = 0
    WHEN_MANOEUVRING = 1
    NEVER = 2
    AFTER_FIRST_CLIMB = 3
    ALWAYS = 4
    GROUND_AND_INFLIGHT = 7


# Values for XKFS.MAG_FUSION
class MagFuseSel():
    NOT_FUSING = 0
    FUSE_YAW = 1
    FUSE_MAG = 2
    FUSE_MAG_ANCHORED = 3


class AutoTestSub(vehicle_test_suite.TestSuite):
    @staticmethod
    def get_not_armable_mode_list():
        return []

    @staticmethod
    def get_not_disarmed_settable_modes_list():
        return []

    @staticmethod
    def get_no_position_not_settable_modes_list():
        return ["AUTO", "GUIDED", "CIRCLE", "POSHOLD"]

    @staticmethod
    def get_position_armable_modes_list():
        return []

    @staticmethod
    def get_normal_armable_modes_list():
        return ["ACRO", "ALT_HOLD", "MANUAL", "STABILIZE", "SURFACE"]

    def log_name(self):
        return "ArduSub"

    def test_filepath(self):
        return os.path.realpath(__file__)

    test_root = "ArduSub_Tests"

    def default_mode(self):
        return 'MANUAL'

    def sitl_start_location(self):
        return SITL_START_LOCATION

    def sitl_start_heading(self):
        return SITL_START_HEADING

    def default_frame(self):
        return 'vectored'

    def is_sub(self):
        return True

    def watch_altitude_maintained(self, delta=0.3, timeout=5.0):
        """Watch and wait for the actual altitude to be maintained

        Keyword Arguments:
            delta {float} -- Maximum altitude range to be allowed from actual point (default: {0.5})
            timeout {float} -- Timeout time in simulation seconds (default: {5.0})

        Raises:
            NotAchievedException: Exception when altitude fails to hold inside the time and
                altitude range
        """
        tstart = self.get_sim_time_cached()
        previous_altitude = self.assert_receive_message('VFR_HUD').alt
        self.progress(f'Altitude to be watched: {previous_altitude}')
        while True:
            m = self.assert_receive_message('VFR_HUD')
            if self.get_sim_time_cached() - tstart > timeout:
                self.progress(f'Altitude hold done: {previous_altitude}')
                return
            if abs(m.alt - previous_altitude) > delta:
                raise NotAchievedException(
                    f"Altitude not maintained: want {previous_altitude:.2f} (+/- {delta:.2f}) got={m.alt:.2f}")

    def dive(self, alt, mode='MANUAL', timeout=120):
        """Dive to a target altitude."""
        self.progress("DIVE")
        self.change_mode(mode)
        if not self.armed():
            self.wait_ready_to_arm()
            self.arm_vehicle()
        pwm = 1300 if self.get_altitude(relative=True) > alt else 1700
        self.set_rc(Joystick.Throttle, pwm)
        self.wait_altitude(altitude_min=alt - 1, altitude_max=alt, relative=True, timeout=timeout)
        self.set_rc(Joystick.Throttle, 1500)
        self.delay_sim_time(1, reason="altitude to stabilise")
        self.progress("DIVE COMPLETE")

    def prepare_synthetic_seafloor_test(self, sea_floor_depth, rf_target, config_bundle=1):
        self.set_parameters({
            "SCR_ENABLE": 1,
            "RNGFND1_TYPE": 36,
            "RNGFND1_ORIENT": 25,
            "RNGFND1_MIN": 0.10,
            "RNGFND1_MAX": 30.00,
            "SCR_USER1": config_bundle,     # Configuration bundle, sets noise and terrain profile
            "SCR_USER2": sea_floor_depth,   # Depth in meters
            "SCR_USER3": 101,               # Output log records
            "SCR_USER4": rf_target,         # Rangefinder target in meters
        })

        self.install_example_script_context("sub_test_synthetic_seafloor.lua")

        # Reboot to enable scripting.
        self.reboot_sitl()
        self.set_rc_default()
        self.wait_ready_to_arm()

    def watch_true_distance_maintained(self, match_distance, delta=0.3, timeout=5.0, final_waypoint=0):
        """Watch and wait for the rangefinder reading to be maintained"""

        def get_true_distance():
            """Return the True distance from the simulated range finder"""
            m_true = self.assert_receive_message('STATUSTEXT', timeout=3.0)
            match = RE_TR_SEARCH.search(m_true.text)
            if not match:
                return None
            return float(match.group(1))

        tstart = self.get_sim_time_cached()
        self.progress(f'Distance to be watched: {match_distance:.2f} (+/- {delta:.2f})')
        max_delta = 0.0

        while True:
            timed_out = self.get_sim_time_cached() - tstart > timeout
            # If final_waypoint>0 then timeout is failure, otherwise success
            if timed_out and final_waypoint > 0:
                raise NotAchievedException(
                    f"Mission not complete: want waypoint {final_waypoint}, "
                    f"only made it to waypoint {self.mav.waypoint_current()}")
            if timed_out:
                self.progress(f'Distance hold done. Max delta:{max_delta:.2f}m')
                return

            true_distance = get_true_distance()
            if true_distance is None:
                continue
            match_delta = abs(true_distance - match_distance)
            if match_delta > max_delta:
                max_delta = match_delta
            if match_delta > delta:
                raise NotAchievedException(
                    f"Distance not maintained: want {match_distance:.2f} (+/- {delta:.2f}) "
                    f"got={true_distance:.2f} ({match_delta:.2f})")
            if final_waypoint > 0:
                if self.mav.waypoint_current() >= final_waypoint:
                    self.progress(f'Distance hold during mission done. Max delta:{max_delta:.2f}m')
                    return

    def fly_mission_NAV_LOITER_TURNS(self, ccw=False, radius_m=15, depth_m=10):
        '''run a mission containing a single LOITER_TURNS item, checking the
        circle is driven in the direction the mission item asks for.  A
        negative loiter radius asks for a counter-clockwise circle'''
        self.wait_ready_to_arm()
        circle_centre_loc = self.home_position_as_location()
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, -depth_m),
            self.create_MISSION_ITEM_INT(
                mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS,
                p1=1,
                p3=-radius_m if ccw else radius_m,
                x=int(circle_centre_loc.lat*1e7),
                y=int(circle_centre_loc.lng*1e7),
                z=-depth_m,
                frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            ),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.arm_vehicle()
        self.change_mode('AUTO')
        self.wait_current_waypoint(2)
        self.wait_circle_direction(circle_centre_loc, radius_m, ccw)
        self.disarm_vehicle(force=True)

    def default_parameter_list(self):
        ret = super(AutoTestSub, self).default_parameter_list()
        ret["FS_GCS_ENABLE"] = 0  # FIXME
        return ret

    def _MAV_CMD_CONDITION_YAW(self, run_cmd):
        self.arm_vehicle()
        self.change_mode('GUIDED')
        for angle in 5, 30, 60, 10:
            angular_rate = 10
            direction = 1
            relative_or_absolute = 0
            run_cmd(
                mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                p1=angle,
                p2=angular_rate,
                p3=direction,
                p4=relative_or_absolute,  # 1 for relative, 0 for absolute
            )
            self.wait_heading(angle, minimum_duration=2)

        self.start_subtest('Relative angle')
        run_cmd(
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            p1=0,
            p2=10,
            p3=1,
            p4=0,  # 1 for relative, 0 for absolute
        )
        self.wait_heading(0, minimum_duration=2)
        run_cmd(
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            p1=20,
            p2=10,
            p3=1,
            p4=1,  # 1 for relative, 0 for absolute
        )
        self.wait_heading(20, minimum_duration=2)

        self.disarm_vehicle()

    def wait_ekf_happy_const_pos(self, timeout=45):
        # All of these must be set for arming to happen in constant position mode:
        required_value = (mavutil.mavlink.EKF_ATTITUDE |
                          mavutil.mavlink.EKF_VELOCITY_HORIZ |
                          mavutil.mavlink.EKF_VELOCITY_VERT |
                          mavutil.mavlink.EKF_POS_VERT_ABS |
                          mavutil.mavlink.EKF_CONST_POS_MODE)

        # None of these bits must be set for arming to happen:
        error_bits = mavutil.mavlink.EKF_UNINITIALIZED

        self.wait_ekf_flags(required_value, error_bits, timeout=timeout)

    def wait_ready_to_arm_const_pos(self, timeout=120):
        self.progress("Waiting for ready to arm (constant position mode)")
        start = self.get_sim_time()
        self.wait_ekf_happy_const_pos(timeout=timeout)
        armable_time = self.get_sim_time() - start
        self.progress(f"Took {armable_time} seconds to become armable")
        self.total_waiting_to_arm_time += armable_time
        self.waiting_to_arm_count += 1

    def tests(self):
        '''return list of all tests'''
        ret = super(AutoTestSub, self).tests()

        ret.extend([
            self.TestLogDownloadMAVProxy,
            self.TestLogDownloadMAVProxyNetwork,
            self.TestLogDownloadLogRestart,
        ])

        ret.extend(self.discovered_tests("Sub"))
        return ret
