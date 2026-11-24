'''
Copter autotest: REQUIRE_LOCATION_FOR_ARMING

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class REQUIRE_LOCATION_FOR_ARMING(TestGroup):
    @testcase(bucket="CopterTests1d")
    def REQUIRE_LOCATION_FOR_ARMING(self):
        '''check AP_Arming::Option::REQUIRE_LOCATION_FOR_ARMING works'''
        self.context_push()
        self.set_parameters({
            "SIM_GPS1_NUMSATS": 3,  # EKF does not like < 6
        })
        self.reboot_sitl()
        self.change_mode('STABILIZE')
        self.wait_prearm_sys_status_healthy()
        self.assert_home_position_not_set()
        self.arm_vehicle()
        self.disarm_vehicle()
        self.change_mode('LOITER')
        self.assert_prearm_failure("waiting for home", other_prearm_failures_fatal=False)

        self.change_mode('STABILIZE')
        self.set_parameters({
            "ARMING_NEED_LOC": 1,
        })
        self.assert_prearm_failure("Need Position Estimate", other_prearm_failures_fatal=False)
        self.context_pop()
        self.reboot_sitl()
