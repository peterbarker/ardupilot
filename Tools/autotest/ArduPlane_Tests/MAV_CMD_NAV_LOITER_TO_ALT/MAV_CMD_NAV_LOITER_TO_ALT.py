'''
Plane autotest: MAV_CMD_NAV_LOITER_TO_ALT

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_NAV_LOITER_TO_ALT(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def MAV_CMD_NAV_LOITER_TO_ALT(self):
        '''test loiter to alt mission item'''
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 30),
            (mavutil.mavlink.MAV_CMD_NAV_LOITER_TO_ALT, 0, 0, 500),
            (mavutil.mavlink.MAV_CMD_NAV_LOITER_TO_ALT, 0, 0, 100),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 800, 0, 0),
        ])
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.wait_altitude(450, 475, relative=True, timeout=600)
        self.wait_altitude(75, 125, relative=True, timeout=600)
        self.wait_current_waypoint(4)
        self.fly_home_land_and_disarm()
