'''
Plane autotest: Hirth

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class Hirth(TestGroup):
    @testcase(bucket="PlaneTests1c")
    def Hirth(self):
        '''Test Hirth EFI'''
        self.EFITest(8, "Hirth", "hirth")
