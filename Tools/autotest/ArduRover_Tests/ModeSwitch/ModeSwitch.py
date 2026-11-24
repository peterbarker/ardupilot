'''
Rover autotest: ModeSwitch

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class ModeSwitch(TestGroup):
    @testcase(bucket="Rover")
    def ModeSwitch(self):
        ''''Set modes via modeswitch'''
        self.set_parameter("MODE_CH", 8)
        self.set_rc(8, 1000)
        # mavutil.mavlink.ROVER_MODE_HOLD:
        self.set_parameter("MODE6", 4)
        # mavutil.mavlink.ROVER_MODE_ACRO
        self.set_parameter("MODE5", 1)
        self.set_rc(8, 1800) # PWM for mode6
        self.wait_mode("HOLD")
        self.set_rc(8, 1700) # PWM for mode5
        self.wait_mode("ACRO")
        self.set_rc(8, 1800) # PWM for mode6
        self.wait_mode("HOLD")
        self.set_rc(8, 1700) # PWM for mode5
        self.wait_mode("ACRO")
