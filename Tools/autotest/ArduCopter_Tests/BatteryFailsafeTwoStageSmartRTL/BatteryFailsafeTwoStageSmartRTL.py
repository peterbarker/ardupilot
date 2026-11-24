'''
Copter autotest: BatteryFailsafeTwoStageSmartRTL

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class BatteryFailsafeTwoStageSmartRTL(TestGroup):
    @testcase(bucket="CopterTests2a")
    def BatteryFailsafeTwoStageSmartRTL(self):
        '''Two stage battery failsafe test with SmartRTL'''
        # TWO STAGE BATTERY FAILSAFE: Trigger low battery condition,
        # then critical battery condition. Verify both SmartRTL
        # actions complete
        self.batt_failsafe_init()
        self.takeoffAndMoveAway()
        self.set_parameter('BATT_FS_LOW_ACT', 3)
        self.set_parameter('BATT_FS_CRT_ACT', 4)
        self.delay_sim_time(10, reason="vehicle to move away")
        self.set_parameter('SIM_BATT_VOLTAGE', 11.4)
        self.wait_statustext("Battery 1 is low", timeout=60)
        self.wait_mode("SMART_RTL")
        self.change_mode("LOITER")
        self.delay_sim_time(10, reason="loiter to stabilise before critical voltage")
        self.set_parameter('SIM_BATT_VOLTAGE', 10.0)
        self.wait_statustext("Battery 1 is critical", timeout=60)
        self.wait_mode("SMART_RTL")
        self.wait_disarmed()
        self.set_parameter('SIM_BATT_VOLTAGE', 12.5)
        self.clear_battery_failsafe()
