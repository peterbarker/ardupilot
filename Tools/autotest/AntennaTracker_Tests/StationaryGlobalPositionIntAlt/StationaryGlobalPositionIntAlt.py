'''
Tracker autotest: StationaryGlobalPositionIntAlt

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import AltFrame

from test_discovery import TestGroup
from test_discovery import testcase


class StationaryGlobalPositionIntAlt(TestGroup):
    @testcase(bucket="Tracker")
    def StationaryGlobalPositionIntAlt(self):
        '''Test GLOBAL_POSITION_INT.alt is correct in stationary mode'''
        # Disable GPS so the tracker stays in stationary mode (stationary=true).
        # With GPS enabled the tracker gets a fix, sets stationary=false, and
        # delegates to the base-class send_global_position_int() which is correct.
        self.set_parameter("GPS1_TYPE", 0)
        self.reboot_sitl()

        # Set a known home position; Tracker::set_home() stores this in
        # current_loc, which the stationary path uses directly.
        home_loc = self.sitl_start_location()
        self.set_home(home_loc)

        self.assert_received_message_field_values("GLOBAL_POSITION_INT", {
            "alt": int(home_loc.get_alt_m(AltFrame.ABSOLUTE) * 1000),
        })
