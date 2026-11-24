'''
Copter autotest: mission_NAV_LOITER_TURNS_off_center

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class mission_NAV_LOITER_TURNS_off_center(TestGroup):
    @testcase(bucket="CopterTests1d")
    def mission_NAV_LOITER_TURNS_off_center(self):
        '''test that loiter turns basically works - copter on edge of circle'''
        self.start_subtest("Start circle when on edge of circle")
        radius_m = 30
        self.wait_ready_to_arm()
        here = self.get_location()
        self.fly_mission_NAV_LOITER_TURNS(
            circle_centre_loc=self.offset_location_ne(here, radius_m, 0),
            radius_m=radius_m,
        )
