'''
Copter autotest: BatteryFailsafeTwoStage

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class BatteryFailsafeTwoStage(TestGroup):
    @testcase(bucket="CopterTests2b")
    def BatteryFailsafeTwoStage(self):
        '''Two stage battery failsafe test with RTL and Land'''
        # TWO STAGE BATTERY FAILSAFE: Trigger low battery condition,
        # then critical battery condition. Verify RTL and Land actions
        # complete.
        self.batt_failsafe_init()
        self.takeoffAndMoveAway()
        self.delay_sim_time(3, reason="vehicle to move away")
        self.set_parameters({
            'BATT_FS_LOW_ACT': 2,
            'BATT_FS_CRT_ACT': 1,
            'SIM_BATT_VOLTAGE': 11.4,
        })
        self.wait_statustext("Battery 1 is low", timeout=60)
        self.wait_mode("RTL")
        self.delay_sim_time(10, reason="RTL to progress")
        self.set_parameter('SIM_BATT_VOLTAGE', 10.0)
        self.wait_statustext("Battery 1 is critical", timeout=60)
        self.wait_mode("LAND")
        self.wait_landed_and_disarmed()
        self.set_parameter('SIM_BATT_VOLTAGE', 12.5)
        self.clear_battery_failsafe()
