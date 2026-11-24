'''
Plane autotest: InertialLabsEAHRS

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class InertialLabsEAHRS(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def InertialLabsEAHRS(self):
        '''Test InertialLabs EAHRS support'''
        self.fly_external_AHRS("ILabs", 5)
