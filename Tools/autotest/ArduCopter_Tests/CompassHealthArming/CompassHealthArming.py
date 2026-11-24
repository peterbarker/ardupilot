'''
Copter autotest: CompassHealthArming

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class CompassHealthArming(TestGroup):
    @testcase(bucket="CopterTests1a")
    def CompassHealthArming(self):
        '''compass-health arming check honours the ARMING_CHECK compass bit'''
        self.wait_ready_to_arm()
        # fail all compasses; the shared pre-arm compass check must
        # refuse arming while compass checking is enabled:
        self.set_parameters({
            "SIM_MAG1_FAIL": 1,
            "SIM_MAG2_FAIL": 1,
            "SIM_MAG3_FAIL": 1,
        })
        self.assert_prearm_failure("Compass 1 not healthy",
                                   other_prearm_failures_fatal=False)
        # an actual arm attempt must also be refused:
        self.assert_arm_failure("Compass 1 not healthy")
        # with the compass arming check disabled the vehicle must arm
        # despite the unhealthy compass; an unconditional arm-time
        # check (removed along with ALLOW_ARM_NO_COMPASS) used to
        # refuse this:
        self.set_parameter("ARMING_SKIPCHK", 1 << 2)
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.disarm_vehicle()
