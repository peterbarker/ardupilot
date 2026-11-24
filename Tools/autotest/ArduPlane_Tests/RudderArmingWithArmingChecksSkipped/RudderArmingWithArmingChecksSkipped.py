'''
Plane autotest: RudderArmingWithArmingChecksSkipped

AP_FLAKE8_CLEAN
'''

import vehicle_test_suite

from test_discovery import TestGroup
from test_discovery import testcase


class RudderArmingWithArmingChecksSkipped(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def RudderArmingWithArmingChecksSkipped(self):
        '''check we can't arm with rudder even if all checks are skipped'''
        self.set_parameters({
            "ARMING_RUDDER": 0,
            "ARMING_SKIPCHK": -1,
            "RC4_REVERSED": 0,
        })
        self.reboot_sitl()
        self.delay_sim_time(5, reason="SITL to initialise after reboot")
        self.set_rc(4, 2000)
        w = vehicle_test_suite.WaitAndMaintainDisarmed(
            self,
            minimum_duration=30,
            timeout=60,
        )
        w.run()
