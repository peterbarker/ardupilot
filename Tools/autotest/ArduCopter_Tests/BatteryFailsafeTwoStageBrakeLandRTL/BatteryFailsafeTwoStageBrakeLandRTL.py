'''
Copter autotest: BatteryFailsafeTwoStageBrakeLandRTL

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class BatteryFailsafeTwoStageBrakeLandRTL(TestGroup):
    @testcase(bucket="CopterTests2b")
    def BatteryFailsafeTwoStageBrakeLandRTL(self):
        '''Two stage battery failsafe: low Brake/Land then critical RTL'''
        self.batt_failsafe_init()
        self.takeoffAndMoveAway()
        self.set_parameters({
            'BATT_FS_LOW_ACT': 7,
            'BATT_FS_CRT_ACT': 2,
            'SIM_BATT_VOLTAGE': 11.4,
        })
        self.wait_statustext("Battery 1 is low", timeout=60)
        self.wait_mode("BRAKE")
        self.set_parameter('SIM_BATT_VOLTAGE', 10.0)
        self.wait_statustext("Battery 1 is critical", timeout=60)
        self.wait_mode("RTL")
        self.wait_landed_and_disarmed()
