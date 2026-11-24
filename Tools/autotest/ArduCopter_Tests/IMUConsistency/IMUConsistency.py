'''
Copter autotest: IMUConsistency

AP_FLAKE8_CLEAN
'''

import math

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class IMUConsistency(TestGroup):
    @testcase(bucket="CopterTests1c")
    def IMUConsistency(self):
        '''test IMUs must be consistent with one another'''
        self.wait_ready_to_arm()

        self.start_subsubtest("prearm checks for accel inconsistency")
        self.context_push()
        self.set_parameters({
            "SIM_ACC1_BIAS_X": 5,
        })
        self.assert_prearm_failure("Accels inconsistent")
        self.context_pop()
        tstart = self.get_sim_time()
        self.wait_ready_to_arm()
        if self.get_sim_time() - tstart < 8:
            raise NotAchievedException("Should take 10 seconds to become armableafter IMU upset")

        self.start_subsubtest("prearm checks for gyro inconsistency")
        self.context_push()
        self.set_parameters({
            "SIM_GYR1_BIAS_X": math.radians(10),
        })
        self.assert_prearm_failure("Gyros inconsistent")
        self.context_pop()
        tstart = self.get_sim_time()
        self.wait_ready_to_arm()
        if self.get_sim_time() - tstart < 8:
            raise NotAchievedException("Should take 10 seconds to become armableafter IMU upset")
