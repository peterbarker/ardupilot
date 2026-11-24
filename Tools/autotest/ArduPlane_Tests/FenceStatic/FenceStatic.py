'''
Plane autotest: FenceStatic

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import Location
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class FenceStatic(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def FenceStatic(self):
        '''Test Basic Fence Functionality'''
        self.progress("Checking for bizarre healthy-when-not-present-or-enabled")
        self.set_parameter("FENCE_TYPE", 4) # Start by only setting polygon fences, otherwise fence will report present
        self.assert_fence_sys_status(False, False, True)
        self.load_fence("CMAC-fence.txt")
        m = self.mav.recv_match(type='FENCE_STATUS', blocking=True, timeout=2)
        if m is not None:
            raise NotAchievedException("Got FENCE_STATUS unexpectedly")
        self.set_parameter("FENCE_ACTION", 0) # report only
        self.assert_fence_sys_status(True, False, True)
        self.set_parameter("FENCE_ACTION", 1) # RTL
        self.assert_fence_sys_status(True, False, True)
        self.do_fence_enable()
        self.assert_fence_sys_status(True, True, True)
        m = self.assert_receive_message('FENCE_STATUS', timeout=2)
        if m.breach_status:
            raise NotAchievedException("Breached fence unexpectedly (%u)" %
                                       (m.breach_status))
        self.do_fence_disable()
        self.assert_fence_sys_status(True, False, True)
        self.set_parameter("FENCE_ACTION", 1)
        self.assert_fence_sys_status(True, False, True)
        self.set_parameter("FENCE_ACTION", 0)
        self.assert_fence_sys_status(True, False, True)
        self.clear_fence()
        if self.get_parameter("FENCE_TOTAL") != 0:
            raise NotAchievedException("Expected zero points remaining")
        self.assert_fence_sys_status(False, False, True)
        self.progress("Trying to enable fence with no points")
        self.do_fence_enable(want_result=mavutil.mavlink.MAV_RESULT_FAILED)

        # test a rather unfortunate behaviour:
        self.progress("Killing a live fence with fence-clear")
        self.load_fence("CMAC-fence.txt")
        self.set_parameter("FENCE_ACTION", 1) # AC_FENCE_ACTION_RTL_AND_LAND == 1. mavutil.mavlink.FENCE_ACTION_RTL == 4
        self.do_fence_enable()
        self.assert_fence_sys_status(True, True, True)
        self.clear_fence()
        self.wait_sensor_state(mavutil.mavlink.MAV_SYS_STATUS_GEOFENCE, False, False, True)
        if self.get_parameter("FENCE_TOTAL") != 0:
            raise NotAchievedException("Expected zero points remaining")
        self.assert_fence_sys_status(False, False, True)
        self.do_fence_disable()

        # ensure that a fence is present if it is tin can, min alt or max alt
        self.progress("Test other fence types (tin-can, min alt, max alt")
        self.set_parameter("FENCE_TYPE", 1) # max alt
        self.assert_fence_sys_status(True, False, True)
        self.set_parameter("FENCE_TYPE", 8) # min alt
        self.assert_fence_sys_status(True, False, True)
        self.set_parameter("FENCE_TYPE", 2) # tin can
        self.assert_fence_sys_status(True, False, True)

        # Test cannot arm if outside of fence and fence is enabled
        self.progress("Test Arming while vehicle below FENCE_ALT_MIN")
        default_fence_alt_min = self.get_parameter("FENCE_ALT_MIN")
        self.set_parameter("FENCE_ALT_MIN", 50)
        self.set_parameter("FENCE_TYPE", 8) # Enables minimum altitude breaches
        self.do_fence_enable()
        self.delay_sim_time(2, reason="breach to propagate") # Allow breach to propagate
        self.assert_fence_enabled()

        self.try_arm(False, "Vehicle breaching Min Alt fence")
        self.do_fence_disable()
        self.set_parameter("FENCE_ALT_MIN", default_fence_alt_min)

        # Test arming outside inclusion zone
        self.progress("Test arming while Vehicle breaching of inclusion zone")
        self.set_parameter("FENCE_TYPE", 4) # Enables polygon fence types
        self.upload_fences_from_locations([(
            mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, [
                Location.latlon_only(1.000, 1.000),
                Location.latlon_only(1.000, 1.001),
                Location.latlon_only(1.001, 1.001),
                Location.latlon_only(1.001, 1.000)
            ]
        )])
        self.delay_sim_time(10, reason="fence check to run") # let fence check run so it loads-from-eeprom
        self.do_fence_enable()
        self.assert_fence_enabled()
        self.delay_sim_time(2, reason="breach to propagate") # Allow breach to propagate
        self.try_arm(False, "Vehicle breaching Polygon fence")
        self.do_fence_disable()
        self.clear_fence()

        self.progress("Test arming while vehicle inside exclusion zone")
        self.set_parameter("FENCE_TYPE", 4) # Enables polygon fence types
        home_loc = self.get_location()
        locs = [
            Location.latlon_only(home_loc.lat - 0.001, home_loc.lng - 0.001),
            Location.latlon_only(home_loc.lat - 0.001, home_loc.lng + 0.001),
            Location.latlon_only(home_loc.lat + 0.001, home_loc.lng + 0.001),
            Location.latlon_only(home_loc.lat + 0.001, home_loc.lng - 0.001),
        ]
        self.upload_fences_from_locations([
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION, locs),
        ])
        self.delay_sim_time(10, reason="fence check to run") # let fence check run so it loads-from-eeprom
        self.do_fence_enable()
        self.assert_fence_enabled()
        self.delay_sim_time(2, reason="breach to propagate") # Allow breach to propagate
        self.try_arm(False, "Vehicle breaching Polygon fence")
        self.do_fence_disable()
