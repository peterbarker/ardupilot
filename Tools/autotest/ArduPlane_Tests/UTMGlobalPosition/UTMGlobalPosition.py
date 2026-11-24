'''
Plane autotest: UTMGlobalPosition

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class UTMGlobalPosition(TestGroup):
    @testcase(bucket="PlaneTests1c")
    def UTMGlobalPosition(self):
        '''test UTM_GLOBAL_POSITION message sending'''
        self.install_terrain_handlers_context()
        self.wait_ready_to_arm()
        m = self.assert_received_message_field_values("UTM_GLOBAL_POSITION", {
            "flight_state": mavutil.mavlink.UTM_FLIGHT_STATE_GROUND,
        }, poll=True)
        if all(b == 0 for b in m.uas_id):
            raise NotAchievedException("UAS ID is all zeros")
        expected_flags = (
            mavutil.mavlink.UTM_DATA_AVAIL_FLAGS_UAS_ID_AVAILABLE |
            mavutil.mavlink.UTM_DATA_AVAIL_FLAGS_POSITION_AVAILABLE |
            mavutil.mavlink.UTM_DATA_AVAIL_FLAGS_ALTITUDE_AVAILABLE |
            mavutil.mavlink.UTM_DATA_AVAIL_FLAGS_RELATIVE_ALTITUDE_AVAILABLE |
            mavutil.mavlink.UTM_DATA_AVAIL_FLAGS_HORIZONTAL_VELO_AVAILABLE |
            mavutil.mavlink.UTM_DATA_AVAIL_FLAGS_VERTICAL_VELO_AVAILABLE
        )
        if m.flags & expected_flags != expected_flags:
            raise NotAchievedException(
                f"Expected flags 0x{expected_flags:x}, got 0x{m.flags:x}")
        self.takeoff(alt=10, mode="TAKEOFF")
        self.assert_received_message_field_values("UTM_GLOBAL_POSITION", {
            "flight_state": mavutil.mavlink.UTM_FLIGHT_STATE_AIRBORNE,
        }, poll=True)
        self.fly_home_land_and_disarm()
