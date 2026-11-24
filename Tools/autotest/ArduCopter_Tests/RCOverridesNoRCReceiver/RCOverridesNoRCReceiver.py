'''
Copter autotest: RCOverridesNoRCReceiver

AP_FLAKE8_CLEAN
'''

import math

from test_discovery import TestGroup
from test_discovery import testcase


class RCOverridesNoRCReceiver(TestGroup):
    @testcase(bucket="CopterTests2a")
    def RCOverridesNoRCReceiver(self):
        '''test RC override timeout with no RC receiver present'''
        self.set_parameters({
            "MAV_GCS_SYSID": 250,
            "SIM_RC_FAIL": 1,  # no-pulses
            "LOG_DISARMED": 1,  # we are timing sensitive, avoid stall on log open
        })
        self.reboot_sitl()

        rc_send_enabled = True
        rc1_value = 1500
        rc2_value = 1500
        rc3_value = 1000
        rc4_value = 1500

        # register a hook which sends rcn_values as overrides
        def rc_override_sender(mav, message):
            if message.get_type() == 'ATTITUDE':
                if not rc_send_enabled:
                    return
                self.mav.mav.rc_channels_override_send(
                    mav.target_system, # target system
                    1, # targe component
                    rc1_value, # chan1_raw
                    rc2_value, # chan2_raw
                    rc3_value, # chan3_raw
                    rc4_value, # chan4_raw
                    65535, # chan5_raw
                    65535, # chan6_raw
                    65535, # chan7_raw
                    65535  # chan8_raw
                )
        self.install_message_hook_context(rc_override_sender)
        self.do_set_mode_via_command_int('LOITER')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.progress("Takeoff throttle")
        rc3_value = 1800
        self.wait_altitude(20, 30, relative=True)
        self.progress("Neutral throttle")
        rc3_value = 1600
        self.progress("yaw ccw")
        rc4_value = 1450
        self.wait_yaw_speed(math.radians(-30), accuracy=5, minimum_duration=10)
        self.progress("Kill overrides and wait for speed to get neutralised")
        rc_send_enabled = False
        self.wait_yaw_speed(0, 1, minimum_duration=10)
        self.do_set_mode_via_command_int('RTL')
        self.wait_disarmed()
