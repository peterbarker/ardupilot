'''
Copter autotest: mission_NAV_LOITER_TURNS

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class mission_NAV_LOITER_TURNS(TestGroup):
    @testcase(bucket="CopterTests2d")
    def mission_NAV_LOITER_TURNS(self):
        '''test that loiter turns basically works'''
        self.fly_mission_NAV_LOITER_TURNS()
