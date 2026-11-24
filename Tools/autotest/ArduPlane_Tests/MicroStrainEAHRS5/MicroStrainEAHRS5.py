'''
Plane autotest: MicroStrainEAHRS5

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class MicroStrainEAHRS5(TestGroup):
    @testcase(bucket="PlaneTests1c")
    def MicroStrainEAHRS5(self):
        '''Test MicroStrain EAHRS series 5 support'''
        self.fly_external_AHRS("MicroStrain5", 2)
