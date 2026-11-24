'''
Copter autotest: MAV_CMD_NAV_RETURN_TO_LAUNCH

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_NAV_RETURN_TO_LAUNCH(TestGroup):
    @testcase(bucket="CopterTests1a")
    def MAV_CMD_NAV_RETURN_TO_LAUNCH(self):
        '''ensure MAV_CMD_NAV_RETURN_TO_LAUNCH via mavlink works'''
        for command in self.run_cmd, self.run_cmd_int:
            self.change_mode('STABILIZE')
            command(mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH)
            self.wait_mode('RTL')
