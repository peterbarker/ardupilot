'''
Copter autotest: MountRCFailAngle

AP_FLAKE8_CLEAN
'''

import vehicle_test_suite

from test_discovery import TestGroup
from test_discovery import testcase


class MountRCFailAngle(TestGroup):
    @testcase(bucket="CopterTests2c")
    def MountRCFailAngle(self):
        '''check mount behaviour when RC input becomes bad'''
        yaw_servo = 7
        self.setup_servo_mount(yaw_servo=yaw_servo)
        self.set_parameters({
            "RC12_OPTION": 214,  # MOUNT1_YAW
        })
        self.reboot_sitl()

        self.start_subtest("Angle test")
        self.set_rc(12, 1200)
        vehicle_test_suite.WaitAndMaintainServoChannelValue(
            self,
            yaw_servo,
            1200,
            minimum_duration=5,
        ).run()
        self.set_parameter("SIM_RC_FAIL", 2)  # 2:All Channels neutral except Throttle is 950us
        # should hold position
        vehicle_test_suite.WaitAndMaintainServoChannelValue(
            self,
            yaw_servo,
            1200,
            minimum_duration=5,
        ).run()
