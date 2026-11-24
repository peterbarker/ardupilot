'''
Copter autotest: RC_OPTIONS_1_FS_THR_ENABLE_0

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class RC_OPTIONS_1_FS_THR_ENABLE_0(TestGroup):
    @testcase(bucket="CopterTests2d")
    def RC_OPTIONS_1_FS_THR_ENABLE_0(self):
        '''check behaviour with unusual RC parameters'''
        self.set_parameters({
            "RC_OPTIONS": 1,
            "FS_THR_ENABLE": 0,
        })
        self.reboot_sitl()
        self.takeoff(10, mode='GUIDED')
        self.wait_yaw_speed(0, minimum_duration=10)
        self.do_land()
