'''
Tracker autotest: MAV_CMD_MISSION_START

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_MISSION_START(TestGroup):
    @testcase(bucket="Tracker")
    def MAV_CMD_MISSION_START(self):
        '''test MAV_CMD_MISSION_START mavlink command'''
        for method in self.run_cmd, self.run_cmd_int:
            self.change_mode(0)  # "MANUAL"
            method(mavutil.mavlink.MAV_CMD_MISSION_START)
            self.wait_mode("AUTO")
