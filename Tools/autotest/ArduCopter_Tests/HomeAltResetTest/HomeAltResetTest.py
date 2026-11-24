'''
Copter autotest: HomeAltResetTest

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class HomeAltResetTest(TestGroup):
    @testcase(bucket="CopterTests1c")
    def HomeAltResetTest(self):
        '''fly mission from cliff top to water, land, then RTL to cliff top'''
        # terrain handler must be running before customise_SITL_commandline so that
        # TERRAIN_REQUESTs from firmware at KalaupapaCliffs are answered immediately.
        self.install_terrain_handlers_context()
        try:
            # wipe=True clears any stale EEPROM state from a previous failed run;
            # a prior failure may leave TERRAIN_ENABLE=1, which causes the firmware to
            # fetch terrain data at boot and block WPNAV parameter responses.
            self.customise_SITL_commandline(["--home", "KalaupapaCliffs"], wipe=True)
            # non-terrain params first; TERRAIN_ENABLE last because enabling it causes a
            # brief firmware pause that drops subsequent PARAM_REQUEST_READ responses
            self.set_parameters({
                "AUTO_OPTIONS": 3,
                "WP_SPD": 10,           # m/s; keeps test duration manageable
                "WP_SPD_DN": 5,         # m/s
                "WP_SPD_UP": 5,         # m/s; RTL initial climb from sea level
                "RTL_ALT_M": 40,        # m above home (cliff top), clears cliff face on return
                "TERRAIN_ENABLE": 1,
                "SIM_TERRAIN": 1,
            })
            self.wait_ready_to_arm()

            # Explicitly fix home to the current position/altitude before arming so
            # that waypoints relative to home are computed from the correct location.
            self.run_cmd(
                mavutil.mavlink.MAV_CMD_DO_SET_HOME,
            )

            # read descent rate parameters so checks below are not hard-coded
            wp_spd_dn = self.get_parameter("WP_SPD_DN")   # high-speed descent (m/s)
            land_spd_ms = self.get_parameter("LAND_SPD_MS")  # final-approach descent (m/s)

            cruise_alt = 40  # m relative to home (cliff top)

            # Phase 1: take off from cliff top, fly north off the cliff edge, land near sea level.
            # All waypoint altitudes are relative to home (cliff top).
            self.start_flying_simple_relhome_mission([
                (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, cruise_alt),
                # fly north clear of the cliff edge; still at cruise_alt above cliff top
                (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 200, 0, cruise_alt),
                # land on the water; NAV_LAND descends to terrain (~640m below home)
                (mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0),
            ])

            # verify vehicle reaches cruise speed on the outbound leg
            self.wait_groundspeed(7, 13, timeout=120)
            # verify high-speed descent phase (above LAND_ALT_LOW_M) at WP_SPD_DN
            self.wait_climbrate(
                -wp_spd_dn - 1,
                -wp_spd_dn + 1,
                minimum_duration=5,
                timeout=300,
            )
            # verify final low-speed approach (below LAND_ALT_LOW_M) at LAND_SPD_MS
            self.wait_climbrate(
                -land_spd_ms - 0.1,
                -land_spd_ms + 0.1,
                minimum_duration=5,
                timeout=120,
            )
            # verify vehicle decelerates before landing
            self.wait_groundspeed(0, 2, minimum_duration=5, timeout=300)
            self.wait_disarmed(timeout=600)

            # Phase 2: re-arm near sea level and climb to cruise_alt above home (a ~640m
            # vertical climb at WP_SPD_UP); then switch to RTL to return to cliff top.
            self.start_flying_simple_relhome_mission([
                (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, cruise_alt),
            ])

            # wait for vehicle to reach cruise_alt above home (cliff top + cruise_alt ASL)
            self.wait_altitude(cruise_alt - 1, cruise_alt + 5, relative=True, timeout=200)

            # RTL_ALT equals cruise_alt, so the vehicle flies directly to the home position
            self.change_mode("RTL")
            # verify vehicle reaches cruise speed on the inbound horizontal leg
            self.wait_groundspeed(7, 13, timeout=120)
            # verify high-speed descent phase (RTL_ALT_M - LAND_ALT_LOW_M) at WP_SPD_DN
            # duration is shorter here (~30 m at WP_SPD_DN) so minimum_duration is smaller
            self.wait_climbrate(
                -wp_spd_dn - 1,
                -wp_spd_dn + 1,
                minimum_duration=2,
                timeout=300,
            )
            # verify final low-speed approach (below LAND_ALT_LOW_M) at LAND_SPD_MS
            self.wait_climbrate(
                -land_spd_ms - 0.1,
                -land_spd_ms + 0.1,
                minimum_duration=5,
                timeout=120,
            )
            # verify vehicle decelerates on arrival at home
            self.wait_groundspeed(0, 2, minimum_duration=5, timeout=300)
            self.wait_rtl_complete(timeout=300)
        finally:
            # reset SITL home back to the default location so the framework's
            # post-test reboot_sitl() location check passes
            self.customise_SITL_commandline([])
