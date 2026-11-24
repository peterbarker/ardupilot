'''
BalanceBot autotest: DriveMission

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class DriveMission(TestGroup):
    @testcase(bucket="BalanceBot")
    def DriveMission(self):
        '''Drive Mission rover1.txt'''
        self.drive_mission("balancebot1.txt", strict=False)
