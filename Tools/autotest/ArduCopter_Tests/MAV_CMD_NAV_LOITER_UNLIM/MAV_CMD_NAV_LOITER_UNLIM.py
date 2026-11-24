'''
Copter autotest: MAV_CMD_NAV_LOITER_UNLIM

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_NAV_LOITER_UNLIM(TestGroup):
    @testcase(bucket="CopterTests1d")
    def MAV_CMD_NAV_LOITER_UNLIM(self):
        '''ensure MAV_CMD_NAV_LOITER_UNLIM via mavlink works'''
        for command in self.run_cmd, self.run_cmd_int:
            self.change_mode('STABILIZE')
            command(mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM)
            self.wait_mode('LOITER')
