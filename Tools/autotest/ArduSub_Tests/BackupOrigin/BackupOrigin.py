'''
Sub autotest: BackupOrigin

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class BackupOrigin(TestGroup):
    @testcase(bucket="Sub")
    def BackupOrigin(self):
        """Test AHRS_ORIGIN_LAT and AHRS_ORIGIN_LON parameters"""

        self.set_parameters({
            'GPS1_TYPE': 0,             # Disable GPS
            'EK3_SRC1_POSXY': 0,        # Make sure EK3_SRC parameters do not refer to GPS
            'EK3_SRC1_VELXY': 0,        # Make sure EK3_SRC parameters do not refer to GPS
            'AHRS_ORIGIN_LAT': 47.607584,
            'AHRS_ORIGIN_LON': -122.343911,
        })
        # the origin statustext is emitted early in the boot - with no
        # GPS configured the EKF does not wait for anything before
        # adopting the recorded origin - so collection must start
        # before the reboot or the text can be missed:
        self.context_collect('STATUSTEXT')
        self.reboot_sitl()

        # Wait for the EKF to be happy in constant position mode
        self.wait_ready_to_arm_const_pos()

        self.wait_statustext('AHRS: using recorded origin', check_context=True)

        # check that origin has been logged
        if not self.current_onboard_log_contains_message('ORGN'):
            raise NotAchievedException("Did not find expected ORGN message")
