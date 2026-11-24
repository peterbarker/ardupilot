'''
Copter autotest: EKFSource

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class EKFSource(TestGroup):
    @testcase(bucket="CopterTests2a")
    def EKFSource(self):
        '''Check EKF Source Prearms work'''
        self.wait_ready_to_arm()

        self.start_subtest("bad yaw source")
        self.set_parameter("EK3_SRC3_YAW", 17)
        self.assert_prearm_failure("Check EK3_SRC3_YAW")

        self.context_push()
        self.start_subtest("missing required yaw source")
        self.set_parameters({
            "EK3_SRC3_YAW": 3, # External Yaw with Compass Fallback
            "COMPASS_USE": 0,
            "COMPASS_USE2": 0,
            "COMPASS_USE3": 0,
        })
        self.assert_prearm_failure("EK3 sources require Compass")
        self.context_pop()
