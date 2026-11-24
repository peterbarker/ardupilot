'''
Copter autotest: BatteryFailsafeTerminate

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class BatteryFailsafeTerminate(TestGroup):
    @testcase(bucket="CopterTests2c")
    def BatteryFailsafeTerminate(self):
        '''Battery failsafe terminate action - copter disarms and crashes'''
        # Trigger low battery condition with failsafe set to terminate. Copter will disarm and crash.
        self.batt_failsafe_init()
        self.takeoffAndMoveAway()
        self.set_parameter('BATT_FS_LOW_ACT', 5)
        self.delay_sim_time(10, reason="vehicle to move away")
        self.set_parameter('SIM_BATT_VOLTAGE', 11.4)
        self.wait_statustext("Battery 1 is low", timeout=60)
        self.wait_disarmed()
        self.reboot_sitl()  # Copter should start at home
