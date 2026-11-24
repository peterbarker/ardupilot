'''
Rover autotest: AHRSOptionsDCMFallback

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class AHRSOptionsDCMFallback(TestGroup):
    @testcase(bucket="Rover")
    def AHRSOptionsDCMFallback(self):
        '''check AHRS_OPTIONS inhibits DCM fallback when EKF lacks height data'''
        self.context_collect('STATUSTEXT')
        self.set_parameters({
            "AHRS_OPTIONS": 3,  # disable DCM fallback
            "EK3_SRC1_VELZ": 3,  # GPS
            "EK3_SRC1_POSZ": 3,  # GPS
            "SIM_GPS1_NUMSATS": 4,  # EKF does not like < 6, origin is never set
        })
        self.reboot_sitl()
        # the EKF can provide attitude but not vertical velocity/position;
        # the AHRS must still use it as DCM fallback is inhibited
        self.wait_statustext("AHRS: EKF3 active", check_context=True, timeout=120)
        # EKF must be reporting a valid attitude but no vertical position
        self.wait_ekf_flags(mavutil.mavlink.ESTIMATOR_ATTITUDE, mavutil.mavlink.ESTIMATOR_POS_VERT_ABS, timeout=30)

        self.start_subtest("DCM fallback used when not inhibited")
        self.context_clear_collection('STATUSTEXT')
        self.set_parameter("AHRS_OPTIONS", 0)
        self.reboot_sitl()
        self.delay_sim_time(60, "allow time for AHRS to select an estimator")
        if self.statustext_in_collections("AHRS: EKF3 active"):
            raise NotAchievedException("AHRS used EKF3 without EKF height data")
