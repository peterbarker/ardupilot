'''
Plane autotest: DO_REPOSITION_mode_change_refused

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class DO_REPOSITION_mode_change_refused(TestGroup):
    @testcase(bucket="PlaneTests1c")
    def DO_REPOSITION_mode_change_refused(self):
        '''DO_REPOSITION must not redirect the vehicle if it cannot change into GUIDED'''
        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 50),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 3000, 0, 50),
        ])
        self.wait_current_waypoint(2, timeout=60)
        self.wait_distance_to_home(300, 1000, timeout=60)

        self.set_parameter("FLTMODE_GCSBLOCK", 1 << 13)  # GUIDED
        target = self.offset_location_ne(self.home_position_as_location(), 0, -1500)
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_DO_REPOSITION,
            p2=mavutil.mavlink.MAV_DO_REPOSITION_FLAGS_CHANGE_MODE,
            p5=int(target.lat * 1e7),
            p6=int(target.lng * 1e7),
            p7=50,
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            want_result=mavutil.mavlink.MAV_RESULT_FAILED,
        )
        self.assert_mode_is('AUTO')

        # still flying the mission leg north, not heading for the reposition target:
        self.wait_and_maintain(
            value_name="TargetBearing",
            target=0,
            current_value_getter=lambda: self.assert_receive_message('NAV_CONTROLLER_OUTPUT').target_bearing,
            validator=lambda value, target: self.heading_delta(value, target) <= 10,
            minimum_duration=10,
            timeout=30,
        )

        self.reboot_sitl(force=True)
