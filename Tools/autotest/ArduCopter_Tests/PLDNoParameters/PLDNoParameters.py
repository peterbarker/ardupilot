'''
Copter autotest: PLDNoParameters

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class PLDNoParameters(TestGroup):
    @testcase(bucket="CopterTests1e")
    def PLDNoParameters(self):
        ''''try enabling PLD with lat/lng/alt all zero'''
        self.set_parameter('SIM_PLD_LAT', 0)
        self.set_parameter('SIM_PLD_LON', 0)
        self.set_parameter('SIM_PLD_HEIGHT', 0)
        self.set_parameter('SIM_PLD_ENABLE', 1)
        self.wait_statustext('Set SIM_PLD_LAT, SIM_PLD_LAT and SIM_PLD_ALT')
