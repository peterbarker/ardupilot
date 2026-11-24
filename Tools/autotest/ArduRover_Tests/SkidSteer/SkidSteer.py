'''
Rover autotest: SkidSteer

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class SkidSteer(TestGroup):
    @testcase(bucket="Rover")
    def SkidSteer(self):
        '''Check skid-steering'''
        self.customise_SITL_commandline([], model="rover-skid")

        self.change_mode("MANUAL")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.progress("get a known heading to avoid worrying about wrap")
        # this is steering-type-two-paddles
        self.set_rc_from_map({1: 1400, 3: 1500})
        self.wait_heading(90)
        self.progress("straighten up")
        self.set_rc_from_map({1: 1500, 3: 1500})
        self.progress("steer one way")
        self.set_rc_from_map({1: 1600, 3: 1400})
        self.wait_heading(120)
        self.progress("steer the other")
        self.set_rc_from_map({1: 1400, 3: 1600})
        self.wait_heading(60)
        self.zero_throttle()
        self.disarm_vehicle()
