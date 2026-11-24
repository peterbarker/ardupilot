'''
Copter autotest: BatteryFailsafeCriticalLanding

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class BatteryFailsafeCriticalLanding(TestGroup):
    @testcase(bucket="CopterTests2a")
    def BatteryFailsafeCriticalLanding(self):
        '''Battery failsafe critical landing not interrupted by RC failure'''
        # Trigger a critical battery condition, which triggers a land
        # mode failsafe. Trigger an RC failure. Verify the RC failsafe
        # is prevented from stopping the low battery landing.
        self.batt_failsafe_init()
        self.takeoffAndMoveAway(100, 50)
        self.set_parameters({
            'FS_OPTIONS': 0,
            'BATT_FS_LOW_ACT': 1,
            'BATT_FS_CRT_ACT': 1,
            'FS_THR_ENABLE': 1,
        })
        self.delay_sim_time(5, reason="vehicle to move away")
        self.set_parameter('SIM_BATT_VOLTAGE', 10.0)
        self.wait_statustext("Battery 1 is critical", timeout=60)
        self.wait_mode("LAND")
        self.delay_sim_time(10, reason="landing to progress")
        self.set_parameter("SIM_RC_FAIL", 1)
        self.delay_sim_time(10, reason="RC failsafe to not interrupt landing")
        self.wait_mode("LAND")
        self.wait_landed_and_disarmed()
        self.set_parameter('SIM_BATT_VOLTAGE', 12.5)
        self.set_parameter("SIM_RC_FAIL", 0)
        self.reboot_sitl()  # reset to starting position
