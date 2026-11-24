'''
Rover autotest: AP_ROVER_AUTO_ARM_ONCE_ENABLED

AP_FLAKE8_CLEAN
'''

import vehicle_test_suite

from test_discovery import TestGroup
from test_discovery import testcase


class AP_ROVER_AUTO_ARM_ONCE_ENABLED(TestGroup):
    @testcase(bucket="Rover")
    def AP_ROVER_AUTO_ARM_ONCE_ENABLED(self):
        '''test Rover arm-once-when-ready behaviour'''
        # do a little dance so we don't arm after setting the parameter:
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.disarm_vehicle()
        self.set_parameter("ARMING_REQUIRE", 3)
        self.reboot_sitl()
        self.wait_armed(timeout=120)
        self.disarm_vehicle()
        vehicle_test_suite.WaitAndMaintainDisarmed(
            self,
            minimum_duration=10,
            timeout=30,
        ).run()
