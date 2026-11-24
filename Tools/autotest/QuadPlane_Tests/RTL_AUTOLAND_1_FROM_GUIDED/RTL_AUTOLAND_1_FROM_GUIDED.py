'''
QuadPlane autotest: RTL_AUTOLAND_1_FROM_GUIDED

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class RTL_AUTOLAND_1_FROM_GUIDED(TestGroup):
    def send_reposition_to_loc(self, loc):
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_DO_REPOSITION,
            0,
            1,  # reposition flags; 1 means "change to guided"
            0,
            0,
            int(loc.lat * 1e7),
            int(loc.lng * 1e7),
            20,    # alt
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        )

    def reposition_to_loc(self, loc, accuracy=100):
        self.send_reposition_to_loc(loc)
        self.wait_location(
            loc,
            accuracy=accuracy,
            minimum_duration=20,
            timeout=120,
            height_accuracy=None,  # loiter altitude behaviour is not under test
        )

    @testcase(bucket="QuadPlaneTests1c")  # as in fly-home then go to landing sequence
    def RTL_AUTOLAND_1_FROM_GUIDED(self):
        '''test behaviour when RTL_AUTOLAND==1 and entering from guided'''

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
        self.set_current_waypoint(0, check_afterwards=False)

        self.change_mode('AUTO')
        self.wait_ready_to_arm()

        here = self.get_location()
        guided_loc = self.offset_location_ne(here, 500, -500)

        self.arm_vehicle()
        self.wait_current_waypoint(3)  # will be 2km North here
        self.reposition_to_loc(guided_loc)
        self.send_cmd_do_set_mode('RTL')

        self.wait_distance_to_home(100, 200, timeout=120)
        self.wait_current_waypoint(7)

        self.fly_home_land_and_disarm()
