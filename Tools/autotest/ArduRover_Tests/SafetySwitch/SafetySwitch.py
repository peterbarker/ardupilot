'''
Rover autotest: SafetySwitch

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class SafetySwitch(TestGroup):
    @testcase(bucket="Rover")
    def SafetySwitch(self):
        '''check safety switch works'''
        self.start_subtest("Make sure we don't start moving when safety switch enabled")
        self.wait_ready_to_arm()
        self.set_rc(1, 2000)
        self.wait_armed()
        self.set_rc(1, 1500)
        self.set_safetyswitch_on()
        self.wait_groundspeed(0, 0.1, minimum_duration=2)
        self.set_safetyswitch_off()
        self.disarm_vehicle()

        self.start_subtest("Make sure we stop moving when safety switch enabled")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.set_rc(3, 1700)
        self.wait_groundspeed(5, 100, minimum_duration=2)
        self.set_safetyswitch_on()
        self.wait_groundspeed(0, 0.1, minimum_duration=2)
        self.set_safetyswitch_off()
        self.wait_groundspeed(5, 100, minimum_duration=2)
        self.set_rc(3, 1500)
        self.disarm_vehicle()
