'''
Copter autotest: ScriptingOSD

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import AutoTestTimeoutException
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class ScriptingOSD(TestGroup):
    @testcase(bucket="CopterTests2b", disabled="Requires SFML which is not available in CI")
    def ScriptingOSD(self):
        '''test OSD scripting with waypoint mission - requires SFML OSD'''
        # This test requires SITL to be built with SFML support:
        #   ./waf configure --board sitl --enable-sfml --sitl-osd
        # Without SFML, the OSD scripting bindings return nil and scripts fail.

        # Check if OSD is compiled in by looking for OSD_TYPE parameter
        try:
            self.get_parameter("OSD_TYPE", timeout=5)
        except NotAchievedException:
            self.progress("OSD not compiled in, skipping test")
            return

        self.context_collect('STATUSTEXT')

        # Install the OSD example script
        self.install_example_script_context('osd.lua')

        # When built with --sitl-osd, OSD_TYPE defaults to 2 and the OSD backend
        # is initialized at process start. The OSD backend cannot be created
        # after boot (OSD_TYPE requires process restart), so if not built with
        # --sitl-osd, the test will detect this and skip gracefully.
        # SIM_SPEEDUP=5 makes the OSD window visible longer for visual testing.
        self.set_parameters({
            "SCR_ENABLE": 1,
            "OSD_TYPE": 2,  # SITL OSD (requires --sitl-osd at configure time)
            "SIM_SPEEDUP": 5,  # Slow enough to see the OSD
        })
        self.reboot_sitl()

        # The script prints "osd not available" when no OSD backend is present
        # (SITL built without --sitl-osd); detect that and skip gracefully.
        try:
            self.wait_statustext("osd not available", timeout=5, check_context=True)
            self.progress("OSD scripting not functional (SFML not enabled), skipping test")
            return
        except AutoTestTimeoutException:
            # no error message means the OSD is working
            pass

        # AUTO_OPTIONS=3 allows arming and taking off in AUTO
        self.set_parameter("AUTO_OPTIONS", 3)

        # fly a mission while the script draws waypoint info to the OSD
        # (type, north_offset_m, east_offset_m, alt_m)
        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 50, 0, 20),    # 50m North
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 50, 50, 20),   # 50m North, 50m East
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 50, 20),    # 50m East
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])

        # Wait for mission to complete (land and disarm)
        self.wait_disarmed(timeout=180)
