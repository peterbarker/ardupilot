'''
Copter autotest: AC_Avoidance_Proximity

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class AC_Avoidance_Proximity(TestGroup):
    @testcase(bucket="CopterTests1a")
    def AC_Avoidance_Proximity(self):
        '''Test proximity avoidance slide behaviour'''

        self.context_push()

        self.load_fence("copter-avoidance-fence.txt")
        self.set_parameters({
            "FENCE_ENABLE": 1,
            "PRX1_TYPE": 10,
            "PRX_LOG_RAW": 1,
            "RC10_OPTION": 40, # proximity-enable
        })
        self.reboot_sitl()
        self.progress("Enabling proximity")
        self.set_rc(10, 2000)
        self.check_avoidance_corners()

        self.assert_current_onboard_log_contains_message("PRX")
        self.assert_current_onboard_log_contains_message("PRXR")

        self.disarm_vehicle(force=True)

        self.context_pop()
        self.reboot_sitl()
