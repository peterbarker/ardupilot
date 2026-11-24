'''
Copter autotest: BatteryFailsafeBrakeLand

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class BatteryFailsafeBrakeLand(TestGroup):
    @testcase(bucket="CopterTests2b")
    def BatteryFailsafeBrakeLand(self):
        '''Battery failsafe brake/land action triggers BRAKE'''
        # Trigger low battery condition with failsafe set to brake/land
        self.batt_failsafe_init()
        self.takeoffAndMoveAway()
        self.set_parameter('BATT_FS_LOW_ACT', 7)
        self.delay_sim_time(10, reason="vehicle to move away")
        self.change_mode('LOITER')
        self.set_rc(1, 2000)
        self.wait_groundspeed(8, 10)
        self.set_parameter('SIM_BATT_VOLTAGE', 11.4)
        self.wait_statustext("Battery 1 is low", timeout=60)
        self.wait_mode('BRAKE')
        self.set_rc(1, 1500)
        self.disarm_vehicle(force=True)
        self.reboot_sitl()  # reset to starting position
