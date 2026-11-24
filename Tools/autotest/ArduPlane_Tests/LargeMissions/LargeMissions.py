'''
Plane autotest: LargeMissions

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class LargeMissions(TestGroup):
    @testcase(bucket="PlaneTests1c")
    def LargeMissions(self):
        '''Test Manipulation of Large missions'''
        self.load_mission("Kingaroy-vlarge.txt", strict=False)
        self.load_mission("Kingaroy-vlarge2.txt", strict=False)
