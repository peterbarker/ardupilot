'''
Helicopter autotest: DDVPTail

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class DDVPTail(TestGroup):
    @testcase(bucket="Helicopter")
    def DDVPTail(self):
        # simple check to ensure servo output to DDVP tail is working
        '''Test DDVP Tail Rotor'''
        self.customise_SITL_commandline(
            [],
            defaults_filepath=self.model_defaults_filepath('heli-ddvptail'),
            model="heli-ddvptail",
            wipe=True,
        )
        self.takeoff(10)
        self.wait_servo_channel_value(7, 2000, timeout=10)
        self.do_RTL()
