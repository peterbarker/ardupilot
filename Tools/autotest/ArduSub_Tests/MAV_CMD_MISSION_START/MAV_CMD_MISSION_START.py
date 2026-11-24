'''
Sub autotest: MAV_CMD_MISSION_START

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_MISSION_START(TestGroup):
    @testcase(bucket="Sub")
    def MAV_CMD_MISSION_START(self):
        '''test handling of MAV_CMD_MISSION_START received via mavlink'''
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2000, 0, 0),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])

        self.wait_ready_to_arm()
        self.arm_vehicle()
        for cmd in self.run_cmd, self.run_cmd_int:
            self.change_mode('CIRCLE')
            cmd(mavutil.mavlink.MAV_CMD_MISSION_START)
            self.assert_mode('AUTO')
        self.disarm_vehicle()
