'''
Copter autotest: mission_NAV_LOITER_TURNS_ccw

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class mission_NAV_LOITER_TURNS_ccw(TestGroup):
    @testcase(bucket="CopterTests2d")
    def mission_NAV_LOITER_TURNS_ccw(self):
        '''test that a negative loiter radius gives a counter-clockwise circle'''
        self.fly_mission_NAV_LOITER_TURNS(ccw=True)
