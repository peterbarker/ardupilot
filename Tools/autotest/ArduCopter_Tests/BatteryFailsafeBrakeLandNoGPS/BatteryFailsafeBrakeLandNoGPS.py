'''
Copter autotest: BatteryFailsafeBrakeLandNoGPS

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class BatteryFailsafeBrakeLandNoGPS(TestGroup):
    @testcase(bucket="CopterTests1a")
    def BatteryFailsafeBrakeLandNoGPS(self):
        '''Battery failsafe brake/land action falls back to LAND without GPS'''
        self.batt_failsafe_init()
        self.takeoffAndMoveAway()
        self.set_parameter('BATT_FS_LOW_ACT', 7)
        self.set_parameter('SIM_GPS1_ENABLE', 0)
        self.delay_sim_time(10, reason="GPS disable to take effect")
        self.set_parameter('SIM_BATT_VOLTAGE', 11.4)
        self.wait_statustext("Battery 1 is low", timeout=60)
        self.wait_mode('LAND')
        self.disarm_vehicle(force=True)
        self.reboot_sitl()  # reset to starting position
