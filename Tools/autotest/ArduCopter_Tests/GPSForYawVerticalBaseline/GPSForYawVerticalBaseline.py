'''
Copter autotest: GPSForYawVerticalBaseline

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class GPSForYawVerticalBaseline(TestGroup):
    @testcase(bucket="CopterTests2d")
    def GPSForYawVerticalBaseline(self):
        '''Moving baseline GPS yaw must be rejected for a vertical baseline'''
        # The moving-baseline GPS reports the heading of the antenna baseline
        # in the horizontal plane, so a baseline with no horizontal separation
        # carries no yaw information: the reported heading is receiver noise.
        # The driver must reject the solution rather than publish it as yaw.
        self.load_default_params_file("copter-gps-for-yaw.parm")
        self.set_parameters({
            "GPS1_POS_X": 0.0, "GPS1_POS_Y": 0.0, "GPS1_POS_Z": -0.45,
            "GPS2_POS_X": 0.0, "GPS2_POS_Y": 0.0, "GPS2_POS_Z": 0.45,
            "SIM_GPS1_POS_X": 0.0, "SIM_GPS1_POS_Y": 0.0, "SIM_GPS1_POS_Z": -0.45,
            "SIM_GPS2_POS_X": 0.0, "SIM_GPS2_POS_Y": 0.0, "SIM_GPS2_POS_Z": 0.45,
        })
        self.reboot_sitl()

        self.wait_gps_fix_type_gte(6, message_type="GPS2_RAW", verbose=True)

        # a yaw field of 0 means the GPS does not provide yaw and 65535 means
        # it is configured for yaw but currently unable to provide it (north
        # is reported as 36000)
        tstart = self.get_sim_time()
        while self.get_sim_time_cached() - tstart < 10:
            m = self.assert_receive_message("GPS2_RAW")
            if m.yaw not in [0, 65535]:
                raise NotAchievedException(
                    "Got GPS yaw %.1f deg from a baseline with no horizontal separation" % (m.yaw * 0.01))
