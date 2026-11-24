'''
Rover autotest: SIMCompare

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class SIMCompare(TestGroup):
    @testcase(bucket="Rover")
    def SIMCompare(self):
        '''compare logged EKF2 and EKF3 estimates against simulator truth'''
        self.set_parameters({
            'AHRS_EKF_TYPE': 3,
            'EK2_ENABLE': 1,
            'EK3_ENABLE': 1,
        })
        self.reboot_sitl()

        # drive a triangle to give the estimators some acceleration
        # and yaw change to track:
        self.change_mode('GUIDED')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        home = self.home_position_as_location()
        self.drive_to_location(self.offset_location_ne(home, 40, 0), timeout=60)
        self.drive_to_location(self.offset_location_ne(home, 40, 40), timeout=60)
        self.drive_to_location(home, timeout=60)
        self.disarm_vehicle()

        self.assert_ekfs_match_sim_state()
