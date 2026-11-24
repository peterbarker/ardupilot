'''
Plane autotest: MicroStrainEAHRS7

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class MicroStrainEAHRS7(TestGroup):
    @testcase(bucket="PlaneTests1c")
    def MicroStrainEAHRS7(self):
        '''Test MicroStrain EAHRS series 7 support'''
        self.fly_external_AHRS("MicroStrain7", 7)
