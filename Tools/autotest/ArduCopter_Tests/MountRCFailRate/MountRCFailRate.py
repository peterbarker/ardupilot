'''
Copter autotest: MountRCFailRate

AP_FLAKE8_CLEAN
'''

import vehicle_test_suite

from test_discovery import TestGroup
from test_discovery import testcase


class MountRCFailRate(TestGroup):
    @testcase(bucket="CopterTests2c")
    def MountRCFailRate(self):
        '''check mount behaviour when RC input becomes bad - rates should zero'''
        yaw_servo = 7
        self.setup_servo_mount(yaw_servo=yaw_servo)
        self.set_parameters({
            "RC12_OPTION": 214,  # MOUNT1_YAW
            "MNT1_RC_RATE": 10,
            "RC12_TRIM": 1600,  # non-neutral during failsafe
        })
        self.reboot_sitl()

        self.set_rc(12, 1200)
        self.delay_sim_time(5, reason="allow some yaw movement to enter log")
        self.context_collect('STATUSTEXT')
        self.set_parameter("SIM_RC_FAIL", 2)  # 2:All Channels neutral except Throttle is 950us
        self.wait_statustext('Radio Failsafe', check_context=True, timeout=10)
        val = self.get_servo_channel_value(yaw_servo)
        # should hold position
        vehicle_test_suite.WaitAndMaintainServoChannelValue(
            self,
            yaw_servo,
            val,
            minimum_duration=5,
        ).run()
