'''
Plane autotest: AeronEAHRS

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class AeronEAHRS(TestGroup):
    @testcase(bucket="PlaneTests1a")
    def AeronEAHRS(self):
        '''Test AeronPlx3 EAHRS support'''
        self.fly_external_AHRS("Aeron-PLX3", 10)
