'''
Sub autotest: SIMCompare

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase

from ardusub import Joystick


class SIMCompare(TestGroup):
    @testcase(bucket="Sub")
    def SIMCompare(self):
        '''compare logged EKF2 and EKF3 estimates against simulator truth'''
        self.set_parameters({
            'AHRS_EKF_TYPE': 3,
            'EK2_ENABLE': 1,
            'EK3_ENABLE': 1,
        })
        self.reboot_sitl()

        # dive and translate to give the estimators something to track:
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.set_rc(Joystick.Throttle, 1600)
        self.set_rc(Joystick.Forward, 1600)
        self.set_rc(Joystick.Lateral, 1550)
        self.wait_distance(30, accuracy=7, timeout=200)
        self.set_rc(Joystick.Yaw, 1550)
        self.wait_heading(0)
        self.set_rc(Joystick.Yaw, 1500)
        self.set_rc(Joystick.Forward, 1500)
        self.set_rc(Joystick.Lateral, 1500)
        self.set_rc(Joystick.Throttle, 1500)
        self.delay_sim_time(2, reason="vehicle to settle")
        self.disarm_vehicle()

        self.assert_ekfs_match_sim_state()
