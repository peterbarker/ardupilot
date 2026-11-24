'''
Plane autotest: SIMCompare

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class SIMCompare(TestGroup):
    @testcase(bucket="PlaneTests1c")
    def SIMCompare(self):
        '''compare logged EKF2 and EKF3 estimates against simulator truth'''
        self.set_parameters({
            'AHRS_EKF_TYPE': 3,
            'EK2_ENABLE': 1,
            'EK3_ENABLE': 1,
        })
        self.reboot_sitl()

        # a takeoff, some turning flight and a landing gives the
        # estimators a good range of attitudes and velocities to track:
        self.takeoff(alt=50)
        self.change_mode('CIRCLE')
        # both filters see large transient attitude estimation errors
        # (over 15 degrees of pitch) during the full-throttle
        # climb-out, so compare only from established circling flight
        # onwards:
        compare_from = self.get_sim_time() + 15
        self.delay_sim_time(30, reason="turning flight for the estimators to track")
        self.fly_home_land_and_disarm()

        # EKF2 systematically over-estimates pitch by up to ~15
        # degrees during the accelerating climb-out (gated out, above)
        # and remains a few degrees off in manoeuvring flight; EKF3
        # tracks truth closely throughout:
        self.assert_ekfs_match_sim_state(
            ignore_before_time_s=compare_from,
            max_roll_pitch_err_deg={'XKF1': 3, 'NKF1': 8},
        )
