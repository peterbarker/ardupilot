'''
Copter autotest: MAV_CMD_DO_SET_HOME_bad_location

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_DO_SET_HOME_bad_location(TestGroup):
    @testcase(bucket="CopterTests2d")
    def MAV_CMD_DO_SET_HOME_bad_location(self):
        '''test setting of bad home location'''
        self.wait_ready_to_arm()
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_SET_HOME,
            p5=190,
            p6=190,
            p7=0,
            want_result=mavutil.mavlink.MAV_RESULT_DENIED,
        )
