'''
Plane autotest: EKF_STATUS_REPORT

AP_FLAKE8_CLEAN
'''

import math

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class EKF_STATUS_REPORT(TestGroup):
    def check_ekf_status_report(self, ahrs_type, m):
        '''return None if EKF_STATUS_REPORT m looks healthy, else a string
        describing the problem'''
        variances = {
            "velocity_variance": m.velocity_variance,
            "pos_horiz_variance": m.pos_horiz_variance,
            "pos_vert_variance": m.pos_vert_variance,
            "compass_variance": m.compass_variance,
            "terrain_alt_variance": m.terrain_alt_variance,
            "airspeed_variance": m.airspeed_variance,
        }
        # all variance fields must be finite and non-negative:
        for name, value in variances.items():
            if not math.isfinite(value):
                return "%s not finite (%s)" % (name, value)
            if value < 0:
                return "%s negative (%f)" % (name, value)
        # the normalised innovation variances should be below 1.0 for a
        # vehicle established in a loiter (1.0 == the maximum
        # inconsistency the filter will accept before rejecting the
        # measurement):
        for name in "velocity_variance", "pos_horiz_variance", "pos_vert_variance":
            if variances[name] > 1.0:
                return "%s too high (%f)" % (name, variances[name])
        # the filter should be healthy and initialised:
        required = (mavutil.mavlink.EKF_ATTITUDE |
                    mavutil.mavlink.EKF_VELOCITY_HORIZ |
                    mavutil.mavlink.EKF_POS_HORIZ_ABS)
        if (m.flags & required) != required:
            return "missing health flags (flags=0x%x)" % m.flags
        if m.flags & mavutil.mavlink.EKF_UNINITIALIZED:
            return "reports EKF_UNINITIALIZED (flags=0x%x)" % m.flags
        return None

    def wait_ekf_status_report_ok(self, ahrs_type, timeout=15):
        '''wait for a fresh EKF_STATUS_REPORT which passes the sanity checks'''
        tstart = self.get_sim_time()
        problem = "no EKF_STATUS_REPORT received"
        while self.get_sim_time_cached() - tstart < timeout:
            m = self.assert_receive_message("EKF_STATUS_REPORT", timeout=5)
            problem = self.check_ekf_status_report(ahrs_type, m)
            if problem is None:
                self.progress("AHRS_EKF_TYPE=%u: EKF_STATUS_REPORT OK (flags=0x%x)" %
                              (ahrs_type, m.flags))
                return
        raise NotAchievedException(
            "AHRS_EKF_TYPE=%u: EKF_STATUS_REPORT not OK: %s" % (ahrs_type, problem))

    @testcase(bucket="PlaneTests1c")
    def EKF_STATUS_REPORT(self):
        '''check EKF_STATUS_REPORT is assembled correctly for each AHRS_EKF_TYPE'''
        # bring up a VectorNav external AHRS on serial4 so that
        # AHRS_EKF_TYPE=11 (EXTERNAL) is a valid selection.  EK2/EK3 are
        # enabled so that AHRS_EKF_TYPE=2 and =3 also have a running
        # backend to report on:
        self.customise_SITL_commandline(["--serial4=sim:VectorNav"])
        self.set_parameters({
            "EAHRS_TYPE": 1,            # VectorNav
            "SERIAL4_PROTOCOL": 36,
            "SERIAL4_BAUD": 230400,
            "GPS1_TYPE": 21,           # GPS from the external AHRS
            "EK2_ENABLE": 1,
            "EK3_ENABLE": 1,
            "AHRS_EKF_TYPE": 11,
            "INS_GYR_CAL": 1,
        })
        self.reboot_sitl()
        self.delay_sim_time(5, reason="external AHRS to initialise")

        self.progress("Running accelcal")
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            p5=4,
            timeout=5,
        )

        # request EKF_STATUS_REPORT often enough to be responsive to
        # AHRS_EKF_TYPE changes:
        self.set_message_rate_hz(mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, 10)

        self.wait_ready_to_arm()
        self.takeoff(70, mode='TAKEOFF')
        self.change_mode('LOITER')
        self.delay_sim_time(10, reason="settle into loiter")

        # 0=DCM (deliberately sends no report), 2=EKF2, 3=EKF3,
        # 10=SIM, 11=EXTERNAL:
        for ahrs_type in [0, 2, 3, 10, 11]:
            self.start_subtest("Checking EKF_STATUS_REPORT for AHRS_EKF_TYPE=%u" % ahrs_type)
            self.set_parameter("AHRS_EKF_TYPE", ahrs_type)
            # allow the configured backend to switch over and produce a
            # fresh report before we look at it:
            self.delay_sim_time(3, reason="AHRS_EKF_TYPE change to take effect")
            self.drain_mav()
            if ahrs_type == 0:
                # DCM does not emit an EKF_STATUS_REPORT
                self.assert_not_receive_message("EKF_STATUS_REPORT", timeout=5)
                continue
            self.wait_ekf_status_report_ok(ahrs_type)

        self.fly_home_land_and_disarm()
