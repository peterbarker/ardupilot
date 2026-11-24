'''
Copter autotest: MAV_CMD_NAV_TAKEOFF

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_NAV_TAKEOFF(TestGroup):
    @testcase(bucket="CopterTests1e")
    def MAV_CMD_NAV_TAKEOFF(self):
        '''test issuing takeoff command via mavlink'''
        self.change_mode('GUIDED')
        self.wait_ready_to_arm()

        self.arm_vehicle()

        self.start_subtest("NAV_VTOL_TAKEOFF must be rejected via COMMAND_LONG")
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_NAV_VTOL_TAKEOFF,
            p7=5,
            want_result=mavutil.mavlink.MAV_RESULT_COMMAND_INT_ONLY,
        )

        self.run_cmd(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, p7=5)
        self.wait_altitude(4.5, 5.5, minimum_duration=5, relative=True)
        self.change_mode('LAND')
        self.wait_disarmed()

        self.start_subtest("Check NAV_TAKEOFF is above home location, not current location")
        # reset home 20 metres above current location
        current_alt_abs = self.get_altitude(relative=False)

        loc = self.get_location()

        home_z_ofs = 20
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_SET_HOME,
            p5=loc.lat,
            p6=loc.lng,
            p7=current_alt_abs + home_z_ofs,
        )

        self.change_mode('GUIDED')
        self.arm_vehicle()
        takeoff_alt = 5
        self.run_cmd(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, p7=takeoff_alt)
        self.wait_altitude(
            current_alt_abs + home_z_ofs + takeoff_alt - 0.5,
            current_alt_abs + home_z_ofs + takeoff_alt + 0.5,
            minimum_duration=5,
            relative=False,
        )
        self.change_mode('LAND')
        self.wait_disarmed()

        self.reboot_sitl()  # unlock home position
