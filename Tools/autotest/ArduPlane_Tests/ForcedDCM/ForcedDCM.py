'''
Plane autotest: ForcedDCM

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class ForcedDCM(TestGroup):
    @testcase(bucket="PlaneTests1a")
    def ForcedDCM(self):
        '''Switch to DCM mid-flight'''
        self.wait_ready_to_arm()
        self.arm_vehicle()

        self.takeoff(50)
        self.context_collect('STATUSTEXT')
        self.set_parameter("AHRS_EKF_TYPE", 0)
        self.wait_statustext("DCM Active", check_context=True)
        self.context_stop_collecting('STATUSTEXT')

        self.change_mode('AUTOLAND')
        self.wait_disarmed(timeout=180)
