'''
Copter autotest: UTMGlobalPositionWaypoint

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class UTMGlobalPositionWaypoint(TestGroup):
    @testcase(bucket="CopterTests1a")
    def UTMGlobalPositionWaypoint(self):
        '''test UTM_GLOBAL_POSITION waypoint fields in AUTO and GUIDED'''
        self.set_parameter("AUTO_OPTIONS", 3)
        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 100, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        # seq 0 = home, seq 1 = TAKEOFF, seq 2 = WAYPOINT (100m north)
        wp = self.assert_fetch_mission_item_int(1, 1, 2, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)

        self.wait_current_waypoint(2, timeout=60)

        # epsilon=1 allows for 1-unit (0.11m) rounding from AP's internal coordinate conversion
        m = self.assert_received_message_field_values("UTM_GLOBAL_POSITION", {
            "next_lat": wp.x,
            "next_lon": wp.y,
        }, poll=True, epsilon=1)
        if not (m.flags & mavutil.mavlink.UTM_DATA_AVAIL_FLAGS_NEXT_WAYPOINT_AVAILABLE):
            raise NotAchievedException(f"AUTO: NEXT_WAYPOINT_AVAILABLE not set (flags=0x{m.flags:x})")
        self.do_RTL()
        self.zero_throttle()

        # GUIDED mode
        self.takeoff(20, mode='GUIDED')
        home = self.poll_message("HOME_POSITION")
        target_lat = home.latitude + 10000
        target_lon = home.longitude
        self.send_set_position_target_global_int(target_lat, target_lon, 20)
        self.delay_sim_time(1, reason="guided target to propagate to UTM message")
        m = self.assert_received_message_field_values("UTM_GLOBAL_POSITION", {
            "next_lat": target_lat,
            "next_lon": target_lon,
        }, poll=True, epsilon=1)
        if not (m.flags & mavutil.mavlink.UTM_DATA_AVAIL_FLAGS_NEXT_WAYPOINT_AVAILABLE):
            raise NotAchievedException(f"GUIDED: NEXT_WAYPOINT_AVAILABLE not set (flags=0x{m.flags:x})")
        self.do_RTL()
