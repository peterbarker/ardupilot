'''
Copter autotest: BatteryFailsafeOptionsContinueLanding

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class BatteryFailsafeOptionsContinueLanding(TestGroup):
    @testcase(bucket="CopterTests1e")
    def BatteryFailsafeOptionsContinueLanding(self):
        '''Battery failsafe with FS_OPTIONS set to continue landing'''
        # Trigger low battery condition in land mode with FS_OPTIONS
        # set to allow land mode to continue. Verify landing completes
        # uninterrupted.
        self.batt_failsafe_init()
        self.takeoffAndMoveAway()
        self.set_parameter('FS_OPTIONS', 8)
        self.change_mode("LAND")
        self.delay_sim_time(5, reason="landing to begin")
        self.set_parameter('SIM_BATT_VOLTAGE', 11.4)
        self.wait_statustext("Battery 1 is low", timeout=60)
        self.delay_sim_time(5, reason="battery failsafe to not interrupt landing")
        self.wait_mode("LAND")
        self.wait_landed_and_disarmed()
        self.set_parameter('SIM_BATT_VOLTAGE', 12.5)
        self.clear_battery_failsafe()

        # we are not at the home location - reboot so the next test starts there
        self.reboot_sitl()
