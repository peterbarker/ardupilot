'''
Copter autotest: BatteryFailsafeDisabledOnGround

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class BatteryFailsafeDisabledOnGround(TestGroup):
    @testcase(bucket="CopterTests2a")
    def BatteryFailsafeDisabledOnGround(self):
        '''Test that with battery failsafe disabled the vehicle is not disarmed on the ground'''
        # Trigger a low battery condition while armed on the ground with
        # the failsafe action set to None.  Verify the vehicle stays armed.
        self.batt_failsafe_init()
        self.set_parameters({
            'DISARM_DELAY': 0,  # stop the auto-disarm-when-landed timer
        })
        self.change_mode('LOITER')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.set_parameter('SIM_BATT_VOLTAGE', 11.4)
        self.wait_statustext("Battery 1 is low", timeout=60)
        self.delay_sim_time(5, reason="battery failsafe to not disarm vehicle")
        self.assert_armed()
        self.set_parameter('SIM_BATT_VOLTAGE', 10.0)
        self.wait_statustext("Battery 1 is critical", timeout=60)
        self.delay_sim_time(5, reason="battery critical failsafe to not disarm vehicle")
        self.assert_armed()
        self.disarm_vehicle()
        self.set_parameter('SIM_BATT_VOLTAGE', 12.5)
        # driving the battery critical latches the failsafe even with the
        # failsafe actions disabled; clear it so a following test can arm
        self.clear_battery_failsafe()
