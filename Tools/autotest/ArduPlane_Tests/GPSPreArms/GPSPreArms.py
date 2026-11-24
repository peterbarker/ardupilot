'''
Plane autotest: GPSPreArms

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class GPSPreArms(TestGroup):
    @testcase(bucket="PlaneTests1a")
    def GPSPreArms(self):
        '''ensure GPS prearm checks work'''
        self.wait_ready_to_arm()
        self.start_subtest('DroneCAN sanity checks')
        self.set_parameter('GPS1_TYPE', 9)
        self.set_parameter('GPS2_TYPE', 9)
        self.set_parameter('GPS1_CAN_OVRIDE', 130)
        self.set_parameter('GPS2_CAN_OVRIDE', 130)
        self.assert_prearm_failure(
            "set for multiple GPS",
            other_prearm_failures_fatal=False,
        )
