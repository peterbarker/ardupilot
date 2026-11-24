'''
Copter autotest: AHRSTrimLand

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class AHRSTrimLand(TestGroup):
    @testcase(bucket="CopterTests1d")
    def AHRSTrimLand(self):
        '''test land detector with significant AHRS trim'''
        self.context_push()
        self.set_parameters({
            "SIM_BRD_TRIM_X": 0.12,
            "AHRS_TRIM_X": 0.12,
        })
        self.reboot_sitl()
        self.wait_ready_to_arm()
        self.takeoff(altitude_min=20, mode='LOITER')
        self.do_RTL()
        self.context_pop()
        self.reboot_sitl()
