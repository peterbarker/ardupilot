'''
Rover autotest: MAV_CMD_MISSION_START

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_MISSION_START(TestGroup):
    @testcase(bucket="Rover")
    def MAV_CMD_MISSION_START(self):
        '''simple test for starting missing using this command'''
        # home and 1 waypoint a long way away:
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2000, 0, 0),
        ])
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        for method in self.run_cmd, self.run_cmd_int:
            self.change_mode('MANUAL')
            self.wait_groundspeed(0, 1)
            method(mavutil.mavlink.MAV_CMD_MISSION_START)
            self.wait_mode('AUTO')
            self.wait_groundspeed(3, 100)
        self.disarm_vehicle()
