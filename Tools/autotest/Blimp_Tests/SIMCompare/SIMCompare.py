'''
Blimp autotest: SIMCompare

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class SIMCompare(TestGroup):
    @testcase(bucket="Blimp")
    def SIMCompare(self):
        '''compare logged EKF2 and EKF3 estimates against simulator truth'''
        self.set_parameters({
            'AHRS_EKF_TYPE': 3,
            'EK2_ENABLE': 1,
            'EK3_ENABLE': 1,
        })
        self.reboot_sitl()

        # loiter and translate to give the estimators something to track:
        self.change_mode('LOITER')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        bl = self.get_location()
        tl = self.offset_location_ne(location=bl, metres_north=5, metres_east=0)
        self.set_rc(2, 2000)
        self.wait_distance_to_location(tl, 0, 0.2, timeout=60)
        self.set_rc(2, 1500)
        self.delay_sim_time(10, reason="vehicle to settle")
        self.disarm_vehicle()

        self.assert_ekfs_match_sim_state()
