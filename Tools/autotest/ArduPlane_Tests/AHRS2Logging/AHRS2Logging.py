'''
Plane autotest: AHRS2Logging

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class AHRS2Logging(TestGroup):
    @testcase(bucket="PlaneTests1a")
    def AHRS2Logging(self):
        '''check the secondary AHRS estimate is logged to the AHR2 message'''
        self.wait_ready_to_arm()
        self.takeoff(alt=100, mode='TAKEOFF')
        self.change_mode('LOITER')
        # loiter long enough to sweep the yaw through a full circle so the
        # secondary yaw and position are meaningfully exercised:
        self.delay_sim_time(60, reason="sweep yaw through a circle")
        self.fly_home_land_and_disarm()
        self.assert_AHR2_log_matches_primary()
