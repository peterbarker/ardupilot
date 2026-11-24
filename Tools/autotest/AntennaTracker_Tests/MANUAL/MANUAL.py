'''
Tracker autotest: MANUAL

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class MANUAL(TestGroup):
    @testcase(bucket="Tracker")
    def MANUAL(self):
        '''Test MANUAL mode'''
        self.change_mode(0) # "MANUAL"
        for chan in 1, 2:
            for pwm in 1200, 1600, 1367:
                self.set_rc(chan, pwm)
                self.wait_servo_channel_value(chan, pwm)
