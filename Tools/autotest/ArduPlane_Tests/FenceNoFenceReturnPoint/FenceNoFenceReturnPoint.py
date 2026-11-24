'''
Plane autotest: FenceNoFenceReturnPoint

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import Location
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class FenceNoFenceReturnPoint(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def FenceNoFenceReturnPoint(self):
        '''Tests calculated return point during fence breach when no fence return point present'''
        """ Attempts to change mode while a fence is breached.
            This should revert to the mode specified by the fence action. """
        want_radius = 100 # Fence Return Radius
        self.set_parameters({
            "FENCE_ACTION": 6,
            "FENCE_TYPE": 4,
            "RTL_RADIUS": want_radius,
            "NAVL1_LIM_BANK": 60,
        })
        home_loc = self.get_location()
        locs = [
            Location.latlon_only(home_loc.lat - 0.003, home_loc.lng - 0.001),
            Location.latlon_only(home_loc.lat - 0.003, home_loc.lng + 0.003),
            Location.latlon_only(home_loc.lat + 0.001, home_loc.lng + 0.003),
            Location.latlon_only(home_loc.lat + 0.001, home_loc.lng - 0.001),
        ]
        self.upload_fences_from_locations([
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, locs),
        ])
        self.delay_sim_time(1, reason="fence upload to complete")
        self.wait_ready_to_arm()
        self.takeoff(alt=50)
        self.set_rc(3, 1500)
        self.change_mode("CRUISE")
        self.wait_distance(150, accuracy=20)

        self.progress("Enable fence and initiate fence action")
        self.do_fence_enable()
        self.assert_fence_enabled()
        self.wait_mode("GUIDED", timeout=120) # We should RTL because of fence breach
        self.delay_sim_time(60, reason="guided fence return to proceed")

        items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        if len(items) != 4:
            raise NotAchievedException("Unexpected fencepoint count (want=%u got=%u)" % (4, len(items)))

        # Check there are no fence return points specified still
        for fence_loc in items:
            if fence_loc.command == mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT:
                raise NotAchievedException(
                    "Unexpected fence return point found (%u) got %u" %
                    (fence_loc.command,
                     mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT))

        # Work out the approximate return point when no fence return point present
        # Logic taken from AC_PolyFence_loader.cpp
        min_loc = self.get_location()
        max_loc = self.get_location()
        for new_loc in locs:
            if new_loc.lat < min_loc.lat:
                min_loc.lat = new_loc.lat
            if new_loc.lng < min_loc.lng:
                min_loc.lng = new_loc.lng
            if new_loc.lat > max_loc.lat:
                max_loc.lat = new_loc.lat
            if new_loc.lng > max_loc.lng:
                max_loc.lng = new_loc.lng

        # Generate the return location based on min and max locs
        ret_lat = (min_loc.lat + max_loc.lat) / 2
        ret_lng = (min_loc.lng + max_loc.lng) / 2
        ret_loc = Location.latlon_only(ret_lat, ret_lng)
        self.progress("Return loc: (%s)" % str(ret_loc))

        # Wait for guided return to vehicle calculated fence return location
        self.wait_distance_to_location(ret_loc, 90, 110)
        self.wait_circling_point_with_radius(ret_loc, 92)

        self.progress("Test complete, disable fence and come home")
        self.do_fence_disable()
        self.fly_home_land_and_disarm()
