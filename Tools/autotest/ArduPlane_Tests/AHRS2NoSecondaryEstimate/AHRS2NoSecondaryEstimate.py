'''
Plane autotest: AHRS2NoSecondaryEstimate

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class AHRS2NoSecondaryEstimate(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def AHRS2NoSecondaryEstimate(self):
        '''check AHRS2 mavlink reporting when the secondary estimator has no
        valid estimate.  Disabling the EKFs leaves DCM as the active
        estimator and the configured-but-unstarted EKF3 as the secondary;
        that secondary has no valid attitude or location.'''
        self.context_push()
        try:
            self.set_parameters({
                "EK2_ENABLE": 0,
                "EK3_ENABLE": 0,
                "AHRS_EKF_TYPE": 3,
            })
            self.reboot_sitl()
            self.delay_sim_time(5, reason="allow DCM to settle")
            # the secondary estimate is now reported even though it has no
            # valid location; AHRS2 is sent regardless of estimate validity:
            self.assert_receive_message('AHRS2', timeout=5)
        finally:
            self.context_pop()
            self.reboot_sitl()
