'''
Plane autotest: FenceDisableUnderAction

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class FenceDisableUnderAction(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def FenceDisableUnderAction(self):
        '''Tests Disabling fence while undergoing action caused by breach'''
        """ Fence breach will cause the vehicle to enter guided mode.
            Upon breach clear, check the vehicle is in the expected mode"""
        self.set_parameters({
            "FENCE_ALT_MIN": 50, # Sets the fence floor
            "FENCE_TYPE": 8,     # Only use fence floor for breaches
        })
        self.wait_ready_to_arm()

        def attempt_fence_breached_disable(start_mode, end_mode, expected_mode, action):
            self.set_parameter("FENCE_ACTION", action)   # Set Fence Action to Guided
            self.change_mode(start_mode)
            self.arm_vehicle()
            self.do_fence_enable()
            self.assert_fence_enabled()
            self.wait_mode(expected_mode)
            self.do_fence_disable()
            self.assert_fence_disabled()
            self.wait_mode(end_mode)
            self.disarm_vehicle(force=True)

        attempt_fence_breached_disable(start_mode="FBWA", end_mode="RTL", expected_mode="RTL", action=1)
        attempt_fence_breached_disable(start_mode="FBWA", end_mode="FBWA", expected_mode="GUIDED", action=6)
        attempt_fence_breached_disable(start_mode="FBWA", end_mode="FBWA", expected_mode="GUIDED", action=7)
