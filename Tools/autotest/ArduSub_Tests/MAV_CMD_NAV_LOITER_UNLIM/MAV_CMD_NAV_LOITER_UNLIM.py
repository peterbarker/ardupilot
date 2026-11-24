'''
Sub autotest: MAV_CMD_NAV_LOITER_UNLIM

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_NAV_LOITER_UNLIM(TestGroup):
    @testcase(bucket="Sub")
    def MAV_CMD_NAV_LOITER_UNLIM(self):
        '''test handling of MAV_CMD_NAV_LOITER_UNLIM received via mavlink'''
        for cmd in self.run_cmd, self.run_cmd_int:
            self.change_mode('CIRCLE')
            cmd(mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM)
            self.assert_mode('POSHOLD')
