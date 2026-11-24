'''
Helicopter autotest: governortest

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class governortest(TestGroup):
    @testcase(bucket="Helicopter")
    def governortest(self):
        '''Test Heli Internal Throttle Curve and Governor'''
        self.customise_SITL_commandline(
            [],
            model="heli-gas",
            wipe=True,
        )
        self.set_parameter("H_RSC_MODE", 4)
        self.takeoff(10)
        self.do_RTL()
