'''
Copter autotest: SIMCompare

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class SIMCompare(TestGroup):
    @testcase(bucket="CopterTests2c")
    def SIMCompare(self):
        '''compare logged EKF2 and EKF3 estimates against simulator truth'''
        self.set_parameters({
            'AHRS_EKF_TYPE': 3,
            'EK2_ENABLE': 1,
            'EK3_ENABLE': 1,
        })
        self.reboot_sitl()

        # fly a mixture of translation, climb, descent and yaw to give
        # the estimators something to track:
        self.change_mode('GUIDED')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.user_takeoff(alt_min=20)
        self.guided_achieve_heading(135)
        self.fly_guided_move_local(30, 40, 15)
        self.guided_achieve_heading(270)
        self.fly_guided_move_local(0, 0, 25)
        self.do_RTL()

        self.assert_ekfs_match_sim_state()
