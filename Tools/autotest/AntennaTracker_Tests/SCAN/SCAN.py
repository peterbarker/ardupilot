'''
Tracker autotest: SCAN

AP_FLAKE8_CLEAN
'''

import operator

from test_discovery import TestGroup
from test_discovery import testcase


class SCAN(TestGroup):
    @testcase(bucket="Tracker")
    def SCAN(self):
        '''Test SCAN mode'''
        self.change_mode(2) # "SCAN"
        self.set_parameter("SCAN_SPEED_YAW", 20)
        for channel in 1, 2:
            self.wait_servo_channel_value(channel,
                                          1900,
                                          timeout=90,
                                          comparator=operator.ge)
        for channel in 1, 2:
            self.wait_servo_channel_value(channel,
                                          1200,
                                          timeout=90,
                                          comparator=operator.le)
