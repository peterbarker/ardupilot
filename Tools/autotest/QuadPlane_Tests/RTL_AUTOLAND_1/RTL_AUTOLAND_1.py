'''
QuadPlane autotest: RTL_AUTOLAND_1

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class RTL_AUTOLAND_1(TestGroup):
    @testcase(bucket="QuadPlaneTests1c")  # as in fly-home then go to landing sequence
    def RTL_AUTOLAND_1(self):
        '''test behaviour when RTL_AUTOLAND==1'''

        self.set_parameters({
            "RTL_AUTOLAND": 1,
        })

        # when RTL is entered and RTL_AUTOLAND is 1 we should fly home
        # then to the landing sequence.  This mission puts the landing
        # sequence well to the West of home so if we go directly there
        # we won't come within 200m of home
        wps = self.create_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 30),
            # fly North
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 500, 0, 30),
            # add a waypoint 1km North (which we will look for and trigger RTL
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 1000, 0, 30),

            # *exciting* landing sequence is ~1km West and points away from Home.
            self.create_MISSION_ITEM_INT(
                mavutil.mavlink.MAV_CMD_DO_LAND_START,
            ),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, -1000, 30),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, -1300, 15),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, -1600, 5),
            (mavutil.mavlink.MAV_CMD_NAV_VTOL_LAND, 0, -1750, 0),
        ])
        self.check_mission_upload_download(wps)

        self.change_mode('AUTO')
        self.wait_ready_to_arm()

        self.arm_vehicle()
        self.wait_current_waypoint(3)  # will be 2km North here
        self.change_mode('RTL')

        self.wait_distance_to_home(100, 200, timeout=120)
        self.wait_current_waypoint(7)

        self.fly_home_land_and_disarm()
