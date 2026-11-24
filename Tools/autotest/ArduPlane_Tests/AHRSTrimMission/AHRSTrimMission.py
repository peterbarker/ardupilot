'''
Plane autotest: AHRSTrimMission

AP_FLAKE8_CLEAN
'''

import math

from pymavlink import mavextra
from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class AHRSTrimMission(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def AHRSTrimMission(self):
        '''fly an AUTO mission (with turns) with a board pitch mounting offset
        corrected by AHRS_TRIM_Y, checking the logged attitude against
        simulator truth in the turns.

        AHRS_TRIM rotates the published attitude from the autopilot-body
        frame into the vehicle-body frame, and that attitude feeds the
        attitude and navigation controllers.  A single-axis AHRS_TRIM_Y is
        common on Plane; when banked the trim mixes into more than one euler
        axis, so compare ATT with SIM (vehicle-body truth) while banked.
        '''
        trim_rad = math.radians(5)
        self.set_parameters({
            "SIM_BRD_TRIM_Y": trim_rad,  # physical board pitch mounting offset
            "AHRS_TRIM_Y": trim_rad,     # corrected by the AHRS trim
        })
        self.reboot_sitl()
        # a square circuit (four turns), then home to auto-land.  The
        # home corner is offset slightly from home because a zero offset
        # is uploaded as zero lat/lng, which Plane takes as the current
        # position:
        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 60),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 400, 0, 60),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 400, 400, 60),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 400, 60),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 10, 10, 60),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 400, 10, 60),
        ])
        # item 0 is home, so the corners are items 2 to 5.  Item 6
        # becomes current as the turn at item 5 begins; wait for that
        # turn onto the northbound leg to finish:
        self.wait_current_waypoint(6, timeout=180)
        self.wait_heading(0, accuracy=10, timeout=60)
        self.fly_home_land_and_disarm(timeout=240)

        # ATT and SIM are written back-to-back in the same logging cycle,
        # so pair each SIM with the ATT preceding it:
        dfreader = self.dfreader_for_current_onboard_log()
        att = None
        count = 0
        pitch_error_sum = 0.0
        yaw_error_sum = 0.0
        while True:
            m = dfreader.recv_match(type=['ATT', 'SIM'])
            if m is None:
                break
            if m.get_type() == 'ATT':
                att = m
                continue
            sim = m
            if att is None or sim.TimeUS - att.TimeUS > 20000:
                continue
            if abs(sim.Roll) < 20:
                continue
            count += 1
            pitch_error_sum += abs(att.Pitch - sim.Pitch)
            yaw_error_sum += abs(mavextra.wrap_180(att.Yaw - sim.Yaw))
        if count < 100:
            raise NotAchievedException(f"Only {count} ATT/SIM samples banked 20deg or more")
        pitch_error = pitch_error_sum / count
        yaw_error = yaw_error_sum / count
        self.progress(f"banked: {count} samples, mean pitch error {pitch_error:.2f}deg, "
                      f"mean yaw error {yaw_error:.2f}deg")
        if pitch_error > 0.75 or yaw_error > 1.5:
            raise NotAchievedException(
                f"Trimmed attitude does not match truth while banked "
                f"(pitch {pitch_error:.2f}deg, yaw {yaw_error:.2f}deg)")
