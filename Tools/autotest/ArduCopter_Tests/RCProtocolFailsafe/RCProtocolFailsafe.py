'''
Copter autotest: RCProtocolFailsafe

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class RCProtocolFailsafe(TestGroup):
    @testcase(bucket="CopterTests2c")
    def RCProtocolFailsafe(self):
        '''ensure we failsafe when the RC protocol failsafe is set'''
        self.takeoff(10, mode='LOITER')
        self.set_parameter('SIM_RC_FAIL', 3)  # protocol failsafe
        self.wait_mode('RTL')
        self.wait_disarmed()
