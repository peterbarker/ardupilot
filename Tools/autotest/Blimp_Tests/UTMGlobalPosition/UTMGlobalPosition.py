'''
Blimp autotest: UTMGlobalPosition

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class UTMGlobalPosition(TestGroup):
    @testcase(bucket="Blimp")
    def UTMGlobalPosition(self):
        '''test UTM_GLOBAL_POSITION message sending'''
        self.wait_ready_to_arm()
        m = self.poll_message("UTM_GLOBAL_POSITION")
        if all(b == 0 for b in m.uas_id):
            raise NotAchievedException("UAS ID is all zeros")
        if m.flags & mavutil.mavlink.UTM_DATA_AVAIL_FLAGS_UAS_ID_AVAILABLE == 0:
            raise NotAchievedException("UAS_ID_AVAILABLE flag not set")
