'''
Copter autotest: CopterMission

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class CopterMission(TestGroup):
    @testcase(bucket="CopterTests1d")
    def CopterMission(self):
        '''fly mission which tests a significant number of commands'''
        # Fly mission #1
        self.progress("# Load copter_mission")
        # load the waypoint count
        num_wp = self.load_mission("copter_mission.txt", strict=False)
        if not num_wp:
            raise NotAchievedException("load copter_mission failed")

        self.fly_loaded_mission(num_wp)

        self.progress("Auto mission completed: passed!")
