'''
Plane autotest: FenceNoFenceReturnPointInclusion

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class FenceNoFenceReturnPointInclusion(TestGroup):
    @testcase(bucket="PlaneTests1c")
    def FenceNoFenceReturnPointInclusion(self):
        '''Tests using home as fence return point when none is present, and no inclusion fence is uploaded'''
        """ Test result when a breach occurs and No fence return point is present and
            no inclusion fence is present and exclusion fence is present """
        want_radius = 100 # Fence Return Radius

        self.set_parameters({
            "FENCE_ACTION": 6,
            "FENCE_TYPE": 2,
            "FENCE_RADIUS": 300,
            "RTL_RADIUS": want_radius,
            "NAVL1_LIM_BANK": 60,
        })

        self.clear_fence()

        self.delay_sim_time(1, reason="fence clear to complete")
        self.wait_ready_to_arm()
        home_loc = self.get_location()
        self.takeoff(alt=50)
        self.set_rc(3, 1500)
        self.change_mode("CRUISE")
        self.wait_distance(150, accuracy=20)

        self.progress("Enable fence and initiate fence action")
        self.do_fence_enable()
        self.assert_fence_enabled()
        self.wait_mode("GUIDED") # We should RTL because of fence breach
        self.delay_sim_time(30, reason="guided fence return to proceed")

        items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        if len(items) != 0:
            raise NotAchievedException("Unexpected fencepoint count (want=%u got=%u)" % (0, len(items)))

        # Check there are no fence return points specified still
        for fence_loc in items:
            if fence_loc.command == mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT:
                raise NotAchievedException(
                    "Unexpected fence return point found (%u) got %u" %
                    (fence_loc.command,
                     mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT))

        # Wait for guided return to vehicle calculated fence return location
        self.wait_distance_to_location(home_loc, 90, 110)
        self.wait_circling_point_with_radius(home_loc, 92)

        self.progress("Test complete, disable fence and come home")
        self.do_fence_disable()
        self.fly_home_land_and_disarm()
