'''
Copter autotest: FlyRangeFinderSITL

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class FlyRangeFinderSITL(TestGroup):
    @testcase(bucket="CopterTests1a")
    def FlyRangeFinderSITL(self):
        '''fly the type=100 perfect rangefinder'''
        self.set_parameters({
            "RNGFND1_TYPE": 100,
            "RTL_ALT_M": 5,
            "RTL_ALT_TYPE": 1,
            "SIM_TERRAIN": 0,
        })
        self.reboot_sitl()
        self.fly_rangefinder_drivers_fly([("unused", "sitl")])
        self.wait_disarmed()
