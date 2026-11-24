'''
Plane autotest: InteractTest

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class InteractTest(TestGroup):
    @testcase(bucket="PlaneTests1b", disabled="requires user interaction")
    def InteractTest(self):
        '''just takeoff'''

        if self.mavproxy is None:
            raise NotAchievedException("Must be started with --map")

        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 30),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 800, 0, 0),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 800, 800, 0),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 400, 0),
        ])

        self.wait_current_waypoint(4)

        self.context_set_speedup(1)

        self.mavproxy.interact()
