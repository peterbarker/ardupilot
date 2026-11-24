'''
Sub autotest: MAV_CMD_DO_CHANGE_SPEED

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_DO_CHANGE_SPEED(TestGroup):
    @testcase(bucket="Sub")
    def MAV_CMD_DO_CHANGE_SPEED(self):
        '''ensure vehicle changes speeds when DO_CHANGE_SPEED received'''
        items = [
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, -3),  # Dive so we have constant drag
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2000, 0, -1),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ]
        self.upload_simple_relhome_mission(items)
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.run_cmd(mavutil.mavlink.MAV_CMD_MISSION_START)
        self.progress("SENT MISSION START")
        self.wait_mode('AUTO')
        self.wait_current_waypoint(2)  # wait after we finish diving to 3m
        for run_cmd in self.run_cmd, self.run_cmd_int:
            for speed in [1, 1.5, 0.5]:
                run_cmd(mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, p2=speed)
                self.wait_groundspeed(speed-0.2, speed+0.2, minimum_duration=2, timeout=60)
        self.disarm_vehicle()
