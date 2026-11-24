'''
Copter autotest: GripperReleaseOnThrustLoss

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class GripperReleaseOnThrustLoss(TestGroup):
    @testcase(bucket="CopterTests2a")
    def GripperReleaseOnThrustLoss(self):
        '''tests that gripper is released on thrust loss if option set'''

        self.context_push()
        self.set_servo_gripper_parameters()
        self.reboot_sitl()

        self.takeoff(30, mode='LOITER')
        self.context_push()
        self.context_collect('STATUSTEXT')
        self.set_parameters({
            "SIM_ENGINE_MUL": 0.5,
            "SIM_ENGINE_FAIL": 1 << 1, # motor 2
            "FLIGHT_OPTIONS": 4,
        })

        self.wait_statustext("Gripper load released", regex=True, timeout=60)
        self.context_pop()

        self.do_RTL()
        self.context_pop()
        self.reboot_sitl()
