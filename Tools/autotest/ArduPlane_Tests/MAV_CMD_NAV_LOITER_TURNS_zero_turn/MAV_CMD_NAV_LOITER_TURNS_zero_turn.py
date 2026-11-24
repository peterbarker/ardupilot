'''
Plane autotest: MAV_CMD_NAV_LOITER_TURNS_zero_turn

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_NAV_LOITER_TURNS_zero_turn(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def MAV_CMD_NAV_LOITER_TURNS_zero_turn(self):
        '''Ensure air vehicle achieves loiter target before exiting'''
        offset = 500
        alt = self.get_parameter("RTL_ALTITUDE")
        waypoint_radius = 100

        home = self.home_position_as_location()
        loiter_turns_loc_ccw = self.offset_location_ne(home, offset, offset)
        loiter_turns_loc_cw = self.offset_location_ne(home, offset, -offset)

        # upload a mission plan containing zero-turn loiters
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 10),
            self.create_MISSION_ITEM_INT(
                mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS,
                p1=0,                # Turns
                p3=-waypoint_radius, # Radius (If positive loiter clockwise, else counter-clockwise)
                x=int(loiter_turns_loc_ccw.lat*1e7),
                y=int(loiter_turns_loc_ccw.lng*1e7),
                z=alt,
                frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
            ),
            self.create_MISSION_ITEM_INT(
                mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS,
                p1=0,               # Turns
                p3=waypoint_radius, # Radius (If positive loiter clockwise, else counter-clockwise)
                x=int(loiter_turns_loc_cw.lat*1e7),
                y=int(loiter_turns_loc_cw.lng*1e7),
                z=alt,
                frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
            ),
            self.create_MISSION_ITEM_INT(
                mavutil.mavlink.MAV_CMD_DO_JUMP,
                p1=2, # waypoint to jump to
                p2=1  # number of jumps (-1: infinite)
            ),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, -offset, -offset, alt),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, -offset, offset, alt),
        ])

        self.change_mode("AUTO")
        self.wait_ready_to_arm()

        self.arm_vehicle()

        # check the vehicle is flying to each waypoint as expected
        self.wait_distance_to_waypoint(2, distance_min=90, distance_max=110, timeout=90) # North East Loiter
        self.wait_distance_to_waypoint(3, distance_min=90, distance_max=110, timeout=90) # North West Loiter

        # Jump to first 0-Turn Loiter
        self.wait_distance_to_waypoint(2, distance_min=90, distance_max=110, timeout=90) # North East Loiter

        # Ensure we go back through this loiter point
        self.wait_distance_to_waypoint(3, distance_min=90, distance_max=110, timeout=90) # North West Loiter

        self.wait_distance_to_waypoint(5, distance_min=10, distance_max=20, timeout=90)  # South West Waypoint
        self.wait_distance_to_waypoint(6, distance_min=10, distance_max=20, timeout=90)  # South East Waypoint

        self.fly_home_land_and_disarm()
