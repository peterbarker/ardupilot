'''
Copter autotest: TakeoffWithLocation

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class TakeoffWithLocation(TestGroup):
    @testcase(bucket="CopterTests2c")
    def TakeoffWithLocation(self):
        '''test we ignore takeoff location'''
        self.change_mode('LOITER')
        self.wait_ready_to_arm()
        arming_loc = self.get_location()
        # the location here should be ignored:
        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 10, 10, 20),
            self.create_MISSION_ITEM_INT(
                mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME,
                p1=10,
                frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            ),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.wait_current_waypoint(2)
        delay_loc = self.get_location()
        if self.get_distance(arming_loc, delay_loc) > 1:
            raise NotAchievedException("Should not move during takeoff")
        self.wait_altitude(18, 21, minimum_duration=10, relative=True)
        self.wait_disarmed()
