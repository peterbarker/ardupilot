'''
QuadPlane autotest: MAV_CMD_NAV_LOITER_TO_ALT

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_NAV_LOITER_TO_ALT(TestGroup):
    @testcase(bucket="QuadPlaneTests1a")
    def MAV_CMD_NAV_LOITER_TO_ALT(self, target_system=1, target_component=1):
        '''ensure consecutive loiter to alts work'''
        self.start_flying_mission('mission.txt')
        self.wait_current_waypoint(4, timeout=240)
        self.assert_altitude(120, accuracy=5, relative=True)
        self.delay_sim_time(30, reason="altitude to remain stable")
        self.assert_altitude(120, accuracy=5, relative=True)
        self.set_current_waypoint(5)
        self.wait_altitude(altitude_min=65, altitude_max=75, relative=True)
        if self.current_waypoint() != 5:
            raise NotAchievedException("Should pass 90m before passing waypoint 5")
        self.wait_disarmed(timeout=300)
