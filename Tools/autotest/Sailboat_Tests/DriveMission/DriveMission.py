'''
Sailboat autotest: DriveMission

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class DriveMission(TestGroup):
    @testcase(bucket="Sailboat")
    def DriveMission(self):
        '''sail a simple mission'''
        self.drive_mission("balancebot1.txt", strict=False)
