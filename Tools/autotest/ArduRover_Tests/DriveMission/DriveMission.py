'''
Rover autotest: DriveMission

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class DriveMission(TestGroup):
    @testcase(bucket="Rover")
    def DriveMission(self):
        '''Drive Mission rover1.txt'''
        self.drive_mission("rover1.txt", strict=False)
