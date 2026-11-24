'''
Plane autotest: MAV_CMD_DO_LAND_START

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_DO_LAND_START(TestGroup):
    @testcase(bucket="PlaneTests1c")
    def MAV_CMD_DO_LAND_START(self):
        '''test MAV_CMD_DO_LAND_START as mavlink command'''
        self.set_parameters({
            "RTL_AUTOLAND": 3,
        })
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 30),
            (mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM, 0, 0, 30),
            self.create_MISSION_ITEM_INT(
                mavutil.mavlink.MAV_CMD_DO_LAND_START,
            ),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 800, 0, 0),
        ])

        self.change_mode('AUTO')
        self.wait_ready_to_arm()

        self.arm_vehicle()

        self.start_subtest("DO_LAND_START as COMMAND_LONG")
        self.wait_current_waypoint(2)
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_LAND_START)
        self.wait_current_waypoint(4)

        self.start_subtest("DO_LAND_START as COMMAND_INT")
        self.set_current_waypoint(2)
        self.run_cmd_int(mavutil.mavlink.MAV_CMD_DO_LAND_START)
        self.wait_current_waypoint(4)

        self.fly_home_land_and_disarm()
