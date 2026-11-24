'''
Sub autotest: DoubleCircle

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class DoubleCircle(TestGroup):
    @testcase(bucket="Sub")
    def DoubleCircle(self):
        '''Test entering circle twice'''
        self.change_mode('CIRCLE')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.change_mode('STABILIZE')
        self.change_mode('CIRCLE')
        self.disarm_vehicle()
