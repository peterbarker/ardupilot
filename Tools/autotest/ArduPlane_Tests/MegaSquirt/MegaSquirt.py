'''
Plane autotest: MegaSquirt

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class MegaSquirt(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def MegaSquirt(self):
        '''test MegaSquirt driver'''
        self.EFITest(
            1, "MegaSquirt", "megasquirt",
            check_fuel_flow=False,
        )
