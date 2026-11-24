'''
Plane autotest: FenceRetRally

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import AltFrame

from test_discovery import TestGroup
from test_discovery import testcase


class FenceRetRally(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def FenceRetRally(self):
        """ Tests the FENCE_RET_RALLY flag, either returning to fence return point,
            or rally point """
        target_system = 1
        target_component = 1
        self.progress("Testing FENCE_ACTION_RTL with fence rally point")

        self.wait_ready_to_arm()

        # Grab a location for fence return point, and upload it.
        fence_loc = self.home_position_as_location()
        self.location_offset_ne(fence_loc, 50, 50)
        fence_return_mission_items = [
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                0, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT,
                0, # current
                0, # autocontinue
                0, # p1
                0, # p2
                0, # p3
                0, # p4
                int(fence_loc.lat * 1e7), # latitude
                int(fence_loc.lng * 1e7), # longitude
                0, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE
            )
        ]
        self.upload_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                                           fence_return_mission_items)
        self.delay_sim_time(1, reason="fence upload to complete")

        # Grab a location for rally point, and upload it.
        rally_loc = self.offset_location_ne(self.home_position_as_location(), -50, 50)
        rally_loc.set_alt_m(15, AltFrame.ABOVE_HOME)
        self.upload_rally_points_from_locations([rally_loc])

        return_radius = 100
        return_alt = 80
        self.set_parameters({
            "RTL_RADIUS": return_radius,
            "FENCE_ACTION": 6, # Set Fence Action to Guided
            "FENCE_TYPE": 8,   # Only use fence floor
            "FENCE_RET_ALT": return_alt,
        })
        self.do_fence_enable()
        self.assert_fence_enabled()

        self.takeoff(alt=15, alt_max=40)
        # Trigger fence breach, fly to rally location
        self.set_parameters({
            "FENCE_RET_RALLY": 1,
            "FENCE_ALT_MIN": 60,
        })
        self.wait_circling_point_with_radius(rally_loc, return_radius)
        self.set_parameter("FENCE_ALT_MIN", 0) # Clear fence breach

        # 10 second fence min retrigger time
        self.delay_sim_time(15, reason="fence retrigger timeout")

        # Fly up before re-triggering fence breach. Fly to fence return point
        self.change_altitude(30, relative=True)
        self.set_parameters({
            "FENCE_RET_RALLY": 0,
            "FENCE_ALT_MIN": 60,
        })
        self.wait_altitude(altitude_min=return_alt-3,
                           altitude_max=return_alt+3,
                           relative=True)
        self.wait_circling_point_with_radius(fence_loc, return_radius)
        self.do_fence_disable() # Disable fence so we can land
        self.fly_home_land_and_disarm() # Pack it up, we're going home.
