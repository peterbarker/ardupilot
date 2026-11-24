'''
Sub autotest: UTMGlobalPositionWaypoint

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class UTMGlobalPositionWaypoint(TestGroup):
    @testcase(bucket="Sub")
    def UTMGlobalPositionWaypoint(self):
        '''test UTM_GLOBAL_POSITION waypoint fields in AUTO'''
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 50, 0, -10),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        # seq 0 = home, seq 1 = WAYPOINT (50m north, 10m depth)
        wp = self.assert_fetch_mission_item_int(1, 1, 1, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)

        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.change_mode('AUTO')
        self.send_cmd(mavutil.mavlink.MAV_CMD_MISSION_START)
        self.wait_current_waypoint(1, timeout=30)

        # epsilon=1 allows for 1-unit (0.11m) rounding from AP's internal coordinate conversion
        m = self.assert_received_message_field_values("UTM_GLOBAL_POSITION", {
            "next_lat": wp.x,
            "next_lon": wp.y,
        }, poll=True, epsilon=1)
        if not (m.flags & mavutil.mavlink.UTM_DATA_AVAIL_FLAGS_NEXT_WAYPOINT_AVAILABLE):
            raise NotAchievedException(f"AUTO: NEXT_WAYPOINT_AVAILABLE not set (flags=0x{m.flags:x})")
        self.disarm_vehicle(force=True)
