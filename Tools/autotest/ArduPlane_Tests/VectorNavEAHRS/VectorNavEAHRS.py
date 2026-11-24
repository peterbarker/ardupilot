'''
Plane autotest: VectorNavEAHRS

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class VectorNavEAHRS(TestGroup):
    @testcase(bucket="PlaneTests1c")
    def VectorNavEAHRS(self):
        '''Test VectorNav EAHRS support'''
        self.fly_external_AHRS("VectorNav", 1)
