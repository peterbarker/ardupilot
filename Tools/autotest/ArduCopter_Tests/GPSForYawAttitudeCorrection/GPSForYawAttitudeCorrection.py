'''
Copter autotest: GPSForYawAttitudeCorrection

AP_FLAKE8_CLEAN
'''

import math

from pymavlink import mavextra

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class GPSForYawAttitudeCorrection(TestGroup):
    @testcase(bucket="CopterTests1a")
    def GPSForYawAttitudeCorrection(self):
        '''Moving baseline GPS yaw must correct for vehicle roll/pitch, incl. under a large board mounting trim'''
        # The moving-baseline GPS reports the antenna baseline heading in the
        # NED horizontal plane.  The GPS backend recovers the vehicle yaw by
        # subtracting the bearing of the body-frame antenna offset, which is
        # only exact when the vehicle is level.  EKF3 applies the residual
        # attitude correction using its own roll/pitch estimate.  We give the
        # baseline a large vertical (Z) component so that a modest vehicle
        # lean swings the apparent horizontal bearing by tens of degrees:
        # without the attitude correction the yaw fed to the EKF is then
        # wrong by a similar amount.
        #
        # A large AHRS_TRIM (board-to-frame mounting offset) is also applied as
        # a frame regression guard.  The EKF works in the autopilot (sensor)
        # body frame and applies the trim only on output; the correction rotates
        # the antenna offset by the EKF's own attitude and fuses the result in
        # that same frame, so the recovered yaw must be unaffected by the trim.
        # A frame slip (e.g. rotating the offset by the published vehicle
        # attitude instead) would bias the yaw, and the large-Z baseline
        # amplifies that bias well past the tolerance below.
        self.load_default_params_file("copter-gps-for-yaw.parm")
        self.set_parameters({
            # Antenna baseline: small fore-aft (X) plus large vertical (Z)
            # separation, no lateral (Y).  When level the baseline is on the
            # nose so the recovered yaw equals the vehicle yaw; under roll the
            # vertical component projects into the horizontal plane.
            "GPS1_POS_X": -0.15, "GPS1_POS_Y": 0.0, "GPS1_POS_Z": -0.45,
            "GPS2_POS_X": 0.15, "GPS2_POS_Y": 0.0, "GPS2_POS_Z": 0.45,
            "SIM_GPS1_POS_X": -0.15, "SIM_GPS1_POS_Y": 0.0, "SIM_GPS1_POS_Z": -0.45,
            "SIM_GPS2_POS_X": 0.15, "SIM_GPS2_POS_Y": 0.0, "SIM_GPS2_POS_Z": 0.45,
            # Remove the simulator's heading lag back-projection so the reported
            # heading reflects the instantaneous attitude.  Otherwise the steady
            # turn rate of the circle would add a lag term the correction does
            # not undo, confounding the comparison against truth.
            "SIM_GPS1_LAG_MS": 0,
            "SIM_GPS2_LAG_MS": 0,
            # large board mounting trim, near the AHRS_TRIM limit: exercises the
            # sensor-vs-vehicle frame handling of the correction (see docstring).
            "AHRS_TRIM_X": 0.1745,
            "AHRS_TRIM_Y": 0.1745,
        })
        self.reboot_sitl()

        self.wait_gps_fix_type_gte(6, message_type="GPS2_RAW", verbose=True)
        self.wait_ready_to_arm()
        self.takeoff(20, mode='GUIDED')

        # fly a gentle circle to hold a steady, modest lean angle.  A large
        # radius keeps the turn rate (and hence any residual yaw lag) small.
        self.set_parameters({
            "CIRCLE_RADIUS_M": 50,
            "CIRCLE_RATE": 12,
        })
        self.change_mode('CIRCLE')

        # Sample the EKF attitude against truth while the vehicle is banked.
        # copter-gps-for-yaw.parm sets EK3_SRC1_YAW=2 so the moving-baseline
        # GPS yaw is the EKF's only yaw source: with the attitude correction
        # in place the EKF yaw tracks truth; without it the fused yaw is wrong
        # by tens of degrees.  Also require the yaw innovation test ratio to
        # stay below 1: a wrong measurement that is merely *rejected* by the
        # innovation gate would leave the EKF yaw temporarily accurate while
        # it coasts on the gyro, which must not count as a pass.
        min_roll_deg = 10
        max_yaw_err_deg = 15
        wanted_samples = 10
        good_samples = 0
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 90:
                raise NotAchievedException(
                    "Only gathered %u/%u banked GPS-yaw samples" % (good_samples, wanted_samples))
            att = self.assert_receive_message("ATTITUDE")
            sim = self.assert_receive_message("SIMSTATE")
            roll_deg = math.degrees(sim.roll)
            if abs(roll_deg) < min_roll_deg:
                continue
            ekf_yaw_deg = math.degrees(att.yaw)
            true_yaw_deg = math.degrees(sim.yaw)
            yaw_err_deg = abs(mavextra.wrap_180(ekf_yaw_deg - true_yaw_deg))
            ekf_status = self.assert_receive_message("EKF_STATUS_REPORT", timeout=10)
            # compass_variance is sqrt(MAX(magTestRatio, yawTestRatio)) and no
            # mag fusion runs with a GPS yaw source, so this recovers the EKF
            # yaw innovation test ratio
            yaw_test_ratio = ekf_status.compass_variance**2
            self.progress("roll=%.1f ekf_yaw=%.1f true_yaw=%.1f err=%.1f yaw_test_ratio=%.2f" %
                          (roll_deg, ekf_yaw_deg, true_yaw_deg, yaw_err_deg, yaw_test_ratio))
            if yaw_err_deg > max_yaw_err_deg:
                raise NotAchievedException(
                    "EKF yaw not corrected for attitude (roll=%.1f deg, yaw err=%.1f deg)" %
                    (roll_deg, yaw_err_deg))
            if yaw_test_ratio > 1.0:
                raise NotAchievedException(
                    "EKF rejecting GPS yaw (roll=%.1f deg, yaw test ratio=%.2f)" %
                    (roll_deg, yaw_test_ratio))
            good_samples += 1
            if good_samples >= wanted_samples:
                break

        self.do_RTL()
