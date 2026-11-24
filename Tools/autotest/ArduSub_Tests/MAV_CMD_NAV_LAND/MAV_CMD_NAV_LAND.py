'''
Sub autotest: MAV_CMD_NAV_LAND

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_NAV_LAND(TestGroup):
    @testcase(bucket="Sub")
    def MAV_CMD_NAV_LAND(self):
        '''test handling of MAV_CMD_NAV_LAND received via mavlink'''
        for cmd in self.run_cmd, self.run_cmd_int:
            self.change_mode('CIRCLE')
            cmd(mavutil.mavlink.MAV_CMD_NAV_LAND)
            self.assert_mode('SURFACE')
