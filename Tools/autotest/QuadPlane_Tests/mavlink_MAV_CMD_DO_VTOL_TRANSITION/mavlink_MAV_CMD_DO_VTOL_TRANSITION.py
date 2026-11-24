'''
QuadPlane autotest: mavlink_MAV_CMD_DO_VTOL_TRANSITION

AP_FLAKE8_CLEAN
'''

import operator

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class mavlink_MAV_CMD_DO_VTOL_TRANSITION(TestGroup):
    @testcase(bucket="QuadPlaneTests1a")
    def mavlink_MAV_CMD_DO_VTOL_TRANSITION(self):
        '''mavlink command forces transition during mission'''
        wps = self.create_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 30),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2000, 0, 30),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.check_mission_upload_download(wps)

        self.change_mode('AUTO')
        self.wait_ready_to_arm()

        self.arm_vehicle()
        self.wait_current_waypoint(2)
        self.wait_servo_channel_value(5, 1000, comparator=operator.eq, timeout=90)

        for command in self.run_cmd, self.run_cmd_int:
            command(mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION, p1=mavutil.mavlink.MAV_VTOL_STATE_MC)
            self.wait_servo_channel_value(5, 1200, comparator=operator.gt, timeout=300)
            command(mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION, p1=mavutil.mavlink.MAV_VTOL_STATE_FW)
            self.wait_servo_channel_value(5, 1000, comparator=operator.eq, timeout=90)

        self.fly_home_land_and_disarm()
