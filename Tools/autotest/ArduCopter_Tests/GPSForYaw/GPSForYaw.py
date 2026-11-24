'''
Copter autotest: GPSForYaw

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class GPSForYaw(TestGroup):
    @testcase(bucket="CopterTests1d")
    def GPSForYaw(self):
        '''Moving baseline GPS yaw'''
        self.load_default_params_file("copter-gps-for-yaw.parm")
        self.reboot_sitl()

        self.wait_gps_fix_type_gte(6, message_type="GPS2_RAW", verbose=True)
        m = self.assert_receive_message("GPS2_RAW", very_verbose=True)
        want = 27000
        if abs(m.yaw - want) > 500:
            raise NotAchievedException("Expected to get GPS-from-yaw (want %f got %f)" % (want, m.yaw))
        self.wait_ready_to_arm()
