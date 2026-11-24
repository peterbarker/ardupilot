'''
Copter autotest: MAV_CMD_SET_EKF_SOURCE_SET

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_SET_EKF_SOURCE_SET(TestGroup):
    def _MAV_CMD_SET_EKF_SOURCE_SET(self, run_cmd):
        run_cmd(
            mavutil.mavlink.MAV_CMD_SET_EKF_SOURCE_SET,
            17,
            want_result=mavutil.mavlink.MAV_RESULT_DENIED,
        )

        self.change_mode('LOITER')
        self.wait_ready_to_arm()

        run_cmd(mavutil.mavlink.MAV_CMD_SET_EKF_SOURCE_SET, 2)

        self.assert_prearm_failure('Need Position Estimate')
        run_cmd(mavutil.mavlink.MAV_CMD_SET_EKF_SOURCE_SET, 1)

        self.wait_ready_to_arm()

    @testcase(bucket="CopterTests1e")
    def MAV_CMD_SET_EKF_SOURCE_SET(self):
        '''test setting of source sets using mavlink command'''
        self._MAV_CMD_SET_EKF_SOURCE_SET(self.run_cmd)
        self._MAV_CMD_SET_EKF_SOURCE_SET(self.run_cmd_int)
