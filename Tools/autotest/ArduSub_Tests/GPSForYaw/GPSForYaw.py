'''
Sub autotest: GPSForYaw

AP_FLAKE8_CLEAN
'''

from math import degrees

from pymavlink import mavextra

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class GPSForYaw(TestGroup):
    @testcase(bucket="Sub")
    def GPSForYaw(self):
        '''Test consumption of heading from NMEA GPS and its propagation to ATTITUDE'''

        self.load_default_params_file("sub-gps-for-yaw-nmea.parm")
        self.reboot_sitl()
        # wait for the vehicle to be ready
        self.wait_ready_to_arm()
        # make sure we are getting both GPS_RAW_INT and SIMSTATE
        simstate_m = self.assert_receive_message("SIMSTATE")
        real_yaw_deg = degrees(simstate_m.yaw)
        expected_yaw_deg = mavextra.wrap_180(real_yaw_deg + 30) # offset in the param file, in degrees
        # wait for GPS_RAW_INT to have a good fix
        self.wait_gps_fix_type_gte(3, message_type="GPS_RAW_INT", verbose=True)

        att_m = self.assert_receive_message("ATTITUDE")
        achieved_yaw_deg = mavextra.wrap_180(degrees(att_m.yaw))

        # ensure new reading propagated to ATTITUDE
        try:
            self.wait_heading(expected_yaw_deg)
        except NotAchievedException as e:
            raise NotAchievedException(
                f"Expected to get yaw consumed and at ATTITUDE (want {expected_yaw_deg} got {achieved_yaw_deg})"
            ) from e
