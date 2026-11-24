'''
Copter autotest: MAV_CMD_NAV_VTOL_LAND

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_NAV_VTOL_LAND(TestGroup):
    @testcase(bucket="CopterTests1e")
    def MAV_CMD_NAV_VTOL_LAND(self):
        '''ensure MAV_CMD_NAV_LAND via mavlink works'''
        for command in self.run_cmd, self.run_cmd_int:
            self.change_mode('STABILIZE')
            command(mavutil.mavlink.MAV_CMD_NAV_VTOL_LAND)
            self.wait_mode('LAND')
            self.change_mode('STABILIZE')
            command(mavutil.mavlink.MAV_CMD_NAV_LAND)
            self.wait_mode('LAND')
