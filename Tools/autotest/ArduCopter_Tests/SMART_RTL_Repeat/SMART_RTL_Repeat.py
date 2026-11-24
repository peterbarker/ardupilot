'''
Copter autotest: SMART_RTL_Repeat

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class SMART_RTL_Repeat(TestGroup):
    @testcase(bucket="CopterTests2b", disabled="Currently fails due to issue with loop detection")
    def SMART_RTL_Repeat(self):
        '''Test whether Smart RTL catches the repeat'''
        self.takeoff(altitude_min=10, mode='GUIDED')
        self.hover()
        self.change_mode("CIRCLE")
        self.delay_sim_time(1300, reason="SmartRTL to fill breadcrumb buffer")
        self.change_mode("SMART_RTL")
        self.wait_disarmed()
