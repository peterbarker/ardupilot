'''
Rover autotest: GPSForYaw

AP_FLAKE8_CLEAN
'''

import math

from pymavlink import mavextra

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class GPSForYaw(TestGroup):
    @testcase(bucket="Rover")
    def GPSForYaw(self):
        '''Test consumption of heading from NMEA GPS and its propagation to ATTITUDE'''

        SIM_GPS1_HDG_OFS = 30.0
        self.set_parameters({
            "EK3_SRC1_YAW": 3,
            "GPS1_TYPE": 5,
            "SIM_GPS1_TYPE": 5,
            "SIM_GPS1_HDG": 1,
            "GPS_AUTO_CONFIG": 0,
            "SIM_GPS1_HDG_OFS": SIM_GPS1_HDG_OFS,
        })
        self.reboot_sitl()
        # wait for the vehicle to be ready
        self.wait_ready_to_arm()
        # make sure we are getting both GPS_RAW_INT and SIMSTATE
        simstate_m = self.assert_receive_message("SIMSTATE")
        real_yaw_deg = math.degrees(simstate_m.yaw)
        expected_yaw_deg = mavextra.wrap_180(real_yaw_deg + SIM_GPS1_HDG_OFS) # offset in the parameters, in degrees
        # wait for GPS_RAW_INT to have a good fix
        self.wait_gps_fix_type_gte(3, message_type="GPS_RAW_INT", verbose=True)

        att_m = self.assert_receive_message("ATTITUDE")
        achieved_yaw_deg = mavextra.wrap_180(math.degrees(att_m.yaw))

        # ensure new reading propagated to ATTITUDE
        try:
            self.wait_heading(expected_yaw_deg)
        except NotAchievedException as e:
            raise NotAchievedException(
                "Expected to get yaw consumed and at ATTITUDE (want %f got %f)" % (expected_yaw_deg, achieved_yaw_deg)
            ) from e
