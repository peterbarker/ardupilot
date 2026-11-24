'''
Rover autotest: BatteryInvalid

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class BatteryInvalid(TestGroup):
    @testcase(bucket="Rover")
    def BatteryInvalid(self):
        '''check Battery prearms report useful data to user'''
        self.start_subtest("Changing battery types makes no difference")
        self.set_parameter("BATT_MONITOR", 0)
        self.assert_prearm_failure("Battery 1 unhealthy", other_prearm_failures_fatal=False)
        self.set_parameter("BATT_MONITOR", 4)
        self.wait_ready_to_arm()

        self.start_subtest("No battery monitor should be armable")
        self.set_parameter("BATT_MONITOR", 0)
        self.reboot_sitl()
        self.wait_ready_to_arm()

        self.set_parameter("BATT_MONITOR", 4)
        self.assert_prearm_failure("Battery 1 unhealthy", other_prearm_failures_fatal=False)
        self.reboot_sitl()
        self.wait_ready_to_arm()

        self.start_subtest("Invalid backend should have a clear error")
        self.set_parameter("BATT_MONITOR", 98)
        self.reboot_sitl()
        self.assert_prearm_failure("Battery 1 unhealthy", other_prearm_failures_fatal=False)

        self.start_subtest("Switching from an invalid backend to a valid backend should require a reboot")
        self.set_parameter("BATT_MONITOR", 4)
        self.assert_prearm_failure("Battery 1 unhealthy", other_prearm_failures_fatal=False)

        self.start_subtest("Switching to None should NOT require a reboot")
        self.set_parameter("BATT_MONITOR", 0)
        self.wait_ready_to_arm()
