'''
Copter autotest: Callisto

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class Callisto(TestGroup):
    @testcase(bucket="CopterTests2a")
    def Callisto(self):
        '''Test Callisto'''
        self.customise_SITL_commandline(
            [],
            **self.callisto_sitl_kwargs()
        )
        self.takeoff(10, mode='GUIDED')
        self.do_RTL()
