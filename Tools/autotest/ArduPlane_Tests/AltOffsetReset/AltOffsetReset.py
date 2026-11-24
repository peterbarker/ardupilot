'''
Plane autotest: AltOffsetReset

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class AltOffsetReset(TestGroup):
    @testcase(bucket="PlaneTests1a")
    def AltOffsetReset(self):
        '''Test automatic ALT_OFFSET reset'''
        reset_alt_offset_option = 1 << 17
        self.change_mode("MANUAL")
        self.set_parameters({
            "FLIGHT_OPTIONS": 0,
            "ALT_OFFSET": 100,
            "SOAR_ENABLE": 0,
        })

        self.change_mode("LOITER")
        self.wait_parameter_value("ALT_OFFSET", 100)

        self.context_collect("STATUSTEXT")
        self.set_parameter("FLIGHT_OPTIONS", reset_alt_offset_option)
        self.run_cmd_do_set_mode(
            "THERMAL",
            want_result=mavutil.mavlink.MAV_RESULT_FAILED,
        )
        self.assert_mode("LOITER")
        self.wait_text("Reset ALT_OFFSET", check_context=True)
        self.wait_parameter_value("ALT_OFFSET", 0)

        self.context_clear_collection("STATUSTEXT")
        self.set_parameter("ALT_OFFSET", 100)
        self.change_mode("MANUAL")
        self.wait_text("Reset ALT_OFFSET", check_context=True)
        self.wait_parameter_value("ALT_OFFSET", 0)

        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 100, 0, 50),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 200, 0, 50),
        ])
        self.change_mode("AUTO")
        self.wait_current_waypoint(1)

        self.context_clear_collection("STATUSTEXT")
        self.set_parameter("ALT_OFFSET", 100)
        self.set_current_waypoint(2)
        self.wait_text("Reset ALT_OFFSET", check_context=True)
        self.wait_parameter_value("ALT_OFFSET", 0)

        # ALT_OFFSET always resets at boot, independently of FLIGHT_OPTIONS.
        self.set_parameters({
            "FLIGHT_OPTIONS": 0,
            "ALT_OFFSET": 100,
        })
        self.context_clear_collection("STATUSTEXT")
        self.reboot_sitl()
        self.wait_parameter_value("ALT_OFFSET", 0)

        # The reset is not saved, so the stored value is reset again next boot.
        self.context_clear_collection("STATUSTEXT")
        self.reboot_sitl()
        self.wait_parameter_value("ALT_OFFSET", 0)
