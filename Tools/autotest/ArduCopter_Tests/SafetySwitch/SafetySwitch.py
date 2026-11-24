'''
Copter autotest: SafetySwitch

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class SafetySwitch(TestGroup):
    @testcase(bucket="CopterTests1a")
    def SafetySwitch(self):
        '''test safety switch behaviour'''
        self.wait_ready_to_arm()

        self.set_safetyswitch_on()
        self.assert_prearm_failure("Safety Switch")

        self.set_safetyswitch_off()
        self.wait_ready_to_arm()

        self.takeoff(2, mode='LOITER')
        self.set_safetyswitch_on()

        self.wait_servo_channel_value(1, 0)
        self.set_safetyswitch_off()

        self.change_mode('LAND')
        self.wait_disarmed()

        # test turning safety on/off using explicit MAVLink command:
        self.run_cmd_int(mavutil.mavlink.MAV_CMD_DO_SET_SAFETY_SWITCH_STATE, mavutil.mavlink.SAFETY_SWITCH_STATE_SAFE)
        self.assert_prearm_failure("Safety Switch")
        self.run_cmd_int(mavutil.mavlink.MAV_CMD_DO_SET_SAFETY_SWITCH_STATE, mavutil.mavlink.SAFETY_SWITCH_STATE_DANGEROUS)
        self.wait_ready_to_arm()
