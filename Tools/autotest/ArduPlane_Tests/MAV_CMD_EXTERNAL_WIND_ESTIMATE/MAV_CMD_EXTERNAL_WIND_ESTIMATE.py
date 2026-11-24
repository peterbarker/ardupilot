'''
Plane autotest: MAV_CMD_EXTERNAL_WIND_ESTIMATE

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_EXTERNAL_WIND_ESTIMATE(TestGroup):
    def _MAV_CMD_EXTERNAL_WIND_ESTIMATE(self, command):
        # the external wind estimate is only used by DCM, and WIND
        # reports the active estimator's wind, so keep DCM active
        # rather than racing EKF3 becoming active after the reboot.
        # DCM's own wind estimator blends the (near-zero) airspeed
        # into its wind on each GPS sample, even on the ground, so
        # stop it using the airspeed sensor or the commanded wind
        # decays before we see it:
        self.set_parameters({
            'AHRS_EKF_TYPE': 0,
            'ARSPD_USE': 0,
        })
        self.reboot_sitl()
        self.wait_gps_fix_type_gte(3)
        self.delay_sim_time(5, reason="let DCM settle after GPS lock")

        def cmp_with_variance(a, b, p):
            return abs(a - b) < p

        def check_eq(speed, direction, ret_dir, timeout=1):
            command(mavutil.mavlink.MAV_CMD_EXTERNAL_WIND_ESTIMATE, p1=speed, p3=direction)

            tstart = self.get_sim_time()
            while True:
                if self.get_sim_time_cached() - tstart > timeout:
                    raise NotAchievedException(
                        f"Failed to set wind speed or/and direction: speed != {speed} or direction != {direction}")

                m = self.assert_receive_message("WIND")
                if cmp_with_variance(m.speed, speed, 0.5) and cmp_with_variance(m.direction, ret_dir, 5):
                    return True

        check_eq(1, 45, 45)
        check_eq(2, 90, 90)
        check_eq(3, 120, 120)
        check_eq(4, 180, -180)
        check_eq(5, 240, -120)
        check_eq(6, 320, -40)
        check_eq(7, 360, 0)

        command(mavutil.mavlink.MAV_CMD_EXTERNAL_WIND_ESTIMATE, p1=-2, p3=90, want_result=mavutil.mavlink.MAV_RESULT_DENIED)
        command(mavutil.mavlink.MAV_CMD_EXTERNAL_WIND_ESTIMATE, p1=2, p3=-90, want_result=mavutil.mavlink.MAV_RESULT_DENIED)
        command(mavutil.mavlink.MAV_CMD_EXTERNAL_WIND_ESTIMATE, p1=-2, p3=-90, want_result=mavutil.mavlink.MAV_RESULT_DENIED)
        command(mavutil.mavlink.MAV_CMD_EXTERNAL_WIND_ESTIMATE, p1=2, p3=370, want_result=mavutil.mavlink.MAV_RESULT_DENIED)

    @testcase(bucket="PlaneTests1c")
    def MAV_CMD_EXTERNAL_WIND_ESTIMATE(self):
        '''test MAV_CMD_EXTERNAL_WIND_ESTIMATE as a mavlink command'''
        self._MAV_CMD_EXTERNAL_WIND_ESTIMATE(self.run_cmd)
        self._MAV_CMD_EXTERNAL_WIND_ESTIMATE(self.run_cmd_int)
