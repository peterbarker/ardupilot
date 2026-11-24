'''
Copter autotest: BatteryFailsafeDisabled

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class BatteryFailsafeDisabled(TestGroup):
    @testcase(bucket="CopterTests2a")
    def BatteryFailsafeDisabled(self):
        '''Test that with battery failsafe disabled no action is taken'''
        # Trigger low battery condition with failsafe disabled. Verify
        # no action taken.
        self.batt_failsafe_init()
        self.takeoffAndMoveAway()
        m = self.assert_receive_message('BATTERY_STATUS')
        if m.charge_state != mavutil.mavlink.MAV_BATTERY_CHARGE_STATE_OK:
            raise NotAchievedException("Expected state ok")
        self.set_parameter('SIM_BATT_VOLTAGE', 11.4)
        self.wait_statustext("Battery 1 is low", timeout=60)
        m = self.assert_receive_message('BATTERY_STATUS')
        if m.charge_state != mavutil.mavlink.MAV_BATTERY_CHARGE_STATE_LOW:
            raise NotAchievedException("Expected state low")
        self.delay_sim_time(5, reason="battery failsafe to not trigger")
        self.wait_mode("ALT_HOLD")
        self.set_parameter('SIM_BATT_VOLTAGE', 10.0)
        self.wait_statustext("Battery 1 is critical", timeout=60)
        m = self.assert_receive_message('BATTERY_STATUS')
        if m.charge_state != mavutil.mavlink.MAV_BATTERY_CHARGE_STATE_CRITICAL:
            raise NotAchievedException("Expected state critical")
        self.delay_sim_time(5, reason="battery critical failsafe to not trigger")
        self.wait_mode("ALT_HOLD")
        self.change_mode("RTL")
        self.wait_rtl_complete()
        self.set_parameter('SIM_BATT_VOLTAGE', 12.5)
        # driving the battery critical latches the failsafe even with the
        # failsafe actions disabled; clear it so a following test can arm
        self.clear_battery_failsafe()
