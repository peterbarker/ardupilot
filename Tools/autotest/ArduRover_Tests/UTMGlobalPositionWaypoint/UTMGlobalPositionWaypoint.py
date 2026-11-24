'''
Rover autotest: UTMGlobalPositionWaypoint

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class UTMGlobalPositionWaypoint(TestGroup):
    @testcase(bucket="Rover")
    def UTMGlobalPositionWaypoint(self):
        '''test UTM_GLOBAL_POSITION waypoint fields in AUTO and GUIDED'''
        self.start_driving_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 50, 0, 0),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        # seq 0 = home, seq 1 = WAYPOINT (50m north)
        wp = self.assert_fetch_mission_item_int(1, 1, 1, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        self.wait_current_waypoint(1, timeout=30)

        # epsilon=1 allows for 1-unit (0.11m) rounding from AP's internal coordinate conversion
        m = self.assert_received_message_field_values("UTM_GLOBAL_POSITION", {
            "next_lat": wp.x,
            "next_lon": wp.y,
        }, poll=True, epsilon=1)
        if not (m.flags & mavutil.mavlink.UTM_DATA_AVAIL_FLAGS_NEXT_WAYPOINT_AVAILABLE):
            raise NotAchievedException(f"AUTO: NEXT_WAYPOINT_AVAILABLE not set (flags=0x{m.flags:x})")
        self.change_mode('HOLD')
        self.disarm_vehicle(force=True)

        # GUIDED mode - DO_REPOSITION uses COMMAND_INT (integer lat/lon, no float precision loss)
        self.change_mode("GUIDED")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        home = self.poll_message("HOME_POSITION")
        target_lat = home.latitude + 10000
        target_lon = home.longitude
        self.send_do_reposition(target_lat, target_lon)
        self.delay_sim_time(1, reason="reposition to register")
        m = self.assert_received_message_field_values("UTM_GLOBAL_POSITION", {
            "next_lat": target_lat,
            "next_lon": target_lon,
        }, poll=True, epsilon=1)
        if not (m.flags & mavutil.mavlink.UTM_DATA_AVAIL_FLAGS_NEXT_WAYPOINT_AVAILABLE):
            raise NotAchievedException(f"GUIDED: NEXT_WAYPOINT_AVAILABLE not set (flags=0x{m.flags:x})")
        self.change_mode('HOLD')
        self.disarm_vehicle(force=True)
