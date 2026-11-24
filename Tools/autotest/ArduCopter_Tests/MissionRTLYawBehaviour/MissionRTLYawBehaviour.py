'''
Copter autotest: MissionRTLYawBehaviour

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class MissionRTLYawBehaviour(TestGroup):
    @testcase(bucket="CopterTests1d")
    def MissionRTLYawBehaviour(self):
        '''check end-of-mission yaw behaviour'''
        self.set_parameters({
            "AUTO_OPTIONS": 3,
        })

        # this checks yaw behaviour by watching the vehicle turn away
        # from where it started, so it needs to start somewhere other
        # than north - the SITL start heading is 270.  The vehicle is
        # left wherever the previous test put it, which can be pointing
        # very nearly north:
        #     MissionRTLYawBehaviour (...) (Bad original heading 1)
        self.reboot_sitl()

        self.start_subtest("behaviour with WP_YAW_BEHAVE set to next-waypoint-except-RTL")
        self.upload_simple_relhome_mission([
            #                                      N   E  U
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,   0, 0, 10),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 20, 0, 10),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        original_heading = self.get_heading()
        if self.heading_delta(original_heading, 0) < 5:
            raise NotAchievedException(f"Bad original heading {original_heading}")
        self.arm_vehicle()
        self.wait_current_waypoint(3)
        self.wait_rtl_complete()
        self.wait_disarmed()
        if self.heading_delta(self.get_heading(), 0) > 5:
            raise NotAchievedException("Should have yaw zero without option")

        # must change out of auto and back in again to reset state machine:
        self.change_mode('LOITER')
        self.change_mode('AUTO')

        self.start_subtest("behaviour with WP_YAW_BEHAVE set to next-waypoint")
        self.upload_simple_relhome_mission([
            #                                      N   E  U
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,   0, 0, 10),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,  0, 20, 20),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.set_parameters({
            "WP_YAW_BEHAVIOR": 1,  # look at next waypoint (including in RTL)
        })
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        original_heading = self.get_heading()
        if self.heading_delta(original_heading, 0) > 1:
            raise NotAchievedException("Bad original heading")
        self.arm_vehicle()
        self.wait_current_waypoint(3)
        self.wait_rtl_complete()
        self.wait_disarmed()
        new_heading = self.get_heading()
        if self.heading_delta(new_heading, original_heading) > 5:
            raise NotAchievedException(f"Should return to original heading want={original_heading} got={new_heading}")
