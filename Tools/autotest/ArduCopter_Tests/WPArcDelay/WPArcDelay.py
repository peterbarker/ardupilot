'''
Copter autotest: WPArcDelay

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class WPArcDelay(TestGroup):
    @testcase(bucket="CopterTests1a")
    def WPArcDelay(self):
        '''test arc waypoints after delays'''
        self.set_parameters({
            "AUTO_OPTIONS": 3,
        })

        pos, radius = self.circle_from_arc((0, 0), (90, 0), 90)
        loc = self.offset_location_ne(self.home_position_as_location(), pos[0], pos[1])

        def fly_and_check_arc(items):
            self.change_mode('STABILIZE')  # needed to ensure Copter mission state machine works
            self.start_flying_simple_relhome_mission(items)
            self.wait_altitude(19, 21, relative=True)
            self.wait_groundspeed(1, 10)
            self.wait_circling_point_with_radius(
                loc, radius,
                epsilon=1,
                min_circle_time=7,
                timeout=30,
                track_angle=False,
            )
            self.wait_distance_to_home(80, 100)
            self.wait_disarmed()

        self.start_subtest("WP -> ARC")
        fly_and_check_arc([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT, 90, 0, 20, {"p1": 90}),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])

        self.start_subtest("WP(delay 5s) -> ARC")
        fly_and_check_arc([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 20, {"p1": 5}),
            (mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT, 90, 0, 20, {"p1": 90}),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])

        self.start_subtest("WP -> NAV_DELAY(5s) -> ARC")
        fly_and_check_arc([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_DELAY, 0, 0, 0, {"p1": 5, "frame": mavutil.mavlink.MAV_FRAME_GLOBAL}),
            (mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT, 90, 0, 20, {"p1": 90}),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])

        self.start_subtest("WP -> CONDITION_DELAY(5s) -> ARC")
        fly_and_check_arc([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_CONDITION_DELAY, 0, 0, 0, {"p1": 5, "frame": mavutil.mavlink.MAV_FRAME_GLOBAL}),
            (mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT, 90, 0, 20, {"p1": 90}),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
