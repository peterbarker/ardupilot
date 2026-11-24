'''
Plane autotest: SimADSB

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class SimADSB(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def SimADSB(self):
        '''Tests to ensure simulated ADSB sensor continues to function'''
        self.set_parameters({
            "SIM_ADSB_COUNT": 1,
            "ADSB_TYPE": 1,
        })
        self.reboot_sitl()
        self.assert_receive_message('ADSB_VEHICLE', timeout=30)
