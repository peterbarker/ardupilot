'''
Plane autotest: MAV_CMD_NAV_RETURN_TO_LAUNCH

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_NAV_RETURN_TO_LAUNCH(TestGroup):
    @testcase(bucket="PlaneTests1c")
    def MAV_CMD_NAV_RETURN_TO_LAUNCH(self):
        '''test receiving MAV_CMD_NAV_RETURN_TO_LAUNCH from GCS'''
        self.set_parameter('RTL_AUTOLAND', 1)
        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 30),
            (mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM, 0, 0, 30),
            self.create_MISSION_ITEM_INT(
                mavutil.mavlink.MAV_CMD_DO_LAND_START,
            ),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 800, 0, 0),
        ])

        for run_cmd in self.run_cmd, self.run_cmd_int:
            self.wait_current_waypoint(2)
            run_cmd(mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH)
            self.wait_current_waypoint(4)
            self.set_current_waypoint(2)
        self.fly_home_land_and_disarm()
