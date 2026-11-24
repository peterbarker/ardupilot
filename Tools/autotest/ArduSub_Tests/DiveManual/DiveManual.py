'''
Sub autotest: DiveManual

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase

from ardusub import Joystick


class DiveManual(TestGroup):
    @testcase(bucket="Sub")
    def DiveManual(self):
        '''Dive manual'''
        self.wait_ready_to_arm()
        self.arm_vehicle()

        self.set_rc(Joystick.Throttle, 1600)
        self.set_rc(Joystick.Forward, 1600)
        self.set_rc(Joystick.Lateral, 1550)

        self.wait_distance(50, accuracy=7, timeout=200)
        self.set_rc(Joystick.Yaw, 1550)

        self.wait_heading(0)
        self.set_rc(Joystick.Yaw, 1500)

        self.wait_distance(50, accuracy=7, timeout=100)
        self.set_rc(Joystick.Yaw, 1550)

        self.wait_heading(0)
        self.set_rc(Joystick.Yaw, 1500)
        self.set_rc(Joystick.Forward, 1500)
        self.set_rc(Joystick.Lateral, 1100)

        self.wait_distance(75, accuracy=7, timeout=100)
        self.set_rc_default()

        self.disarm_vehicle()
        self.progress("Manual dive OK")

        m = self.assert_receive_message('SCALED_PRESSURE3')

        # Note this temperature matches the output of the Atmospheric Model for Air currently
        # And should be within 1 deg C of 40 degC
        if (m.temperature < 3900) or (4100 < m.temperature):
            raise NotAchievedException(f"Did not get correct TSYS01 temperature: Got {m.temperature}")
