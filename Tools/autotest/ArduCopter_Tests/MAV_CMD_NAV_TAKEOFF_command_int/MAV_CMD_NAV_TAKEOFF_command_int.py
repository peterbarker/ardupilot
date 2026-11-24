'''
Copter autotest: MAV_CMD_NAV_TAKEOFF_command_int

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_NAV_TAKEOFF_command_int(TestGroup):
    @testcase(bucket="CopterTests1e")
    def MAV_CMD_NAV_TAKEOFF_command_int(self):
        '''test issuing takeoff command via mavlink and command_int'''
        self.change_mode('GUIDED')
        self.wait_ready_to_arm()

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
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            p7=takeoff_alt,
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        )
        self.wait_altitude(
            current_alt_abs + home_z_ofs + takeoff_alt - 0.5,
            current_alt_abs + home_z_ofs + takeoff_alt + 0.5,
            minimum_duration=5,
            relative=False,
        )
        self.change_mode('LAND')
        self.wait_disarmed()

        self.reboot_sitl()  # unlock home position

        self.start_subtest("NAV_VTOL_TAKEOFF via COMMAND_INT")
        self.change_mode('GUIDED')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_NAV_VTOL_TAKEOFF,
            p7=takeoff_alt,
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        )
        self.wait_altitude(
            takeoff_alt - 0.5,
            takeoff_alt + 0.5,
            minimum_duration=5,
            relative=True,
        )
        self.change_mode('LAND')
        self.wait_disarmed()
