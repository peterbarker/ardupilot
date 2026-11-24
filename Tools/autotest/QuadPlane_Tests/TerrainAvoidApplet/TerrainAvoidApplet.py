'''
QuadPlane autotest: TerrainAvoidApplet

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import AltFrame
from vehicle_test_suite import Location
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class TerrainAvoidApplet(TestGroup):
    def terrain_avoid_applet_setup(self, cmtc_enable):
        '''load quadplane_terrain_avoid.lua at the Top of the World, check we
        have terrain data there, and leave the vehicle in AUTO with the
        mission loaded and the applet activated'''
        self.start_subtest("Terrain Avoidance Load and Start")

        # We do this in a real-world scenario in Alaska where we take off from the Top of the World
        # and fly a mission that goes down into the valley to purposefully trigger Pitcing, Quading and CMTC events
        topofworld_loc = Location(64.1624778, -139.8402246, 1109.0, AltFrame.ABSOLUTE)
        topofworld_alt_amsl = topofworld_loc.get_alt_m(AltFrame.ABSOLUTE)
        self.customise_SITL_commandline(
            ["--home", f"{topofworld_loc.lat},{topofworld_loc.lng},{topofworld_alt_amsl},0"]
        )

        self.context_collect("STATUSTEXT")

        # want 30m STRM data. This has to be set before install_terrain_handlers_context() is called
        self.set_parameters({
            "TERRAIN_ENABLE": 1,
            "TERRAIN_SPACING": 30,
            "TERRAIN_FOLLOW": 1,
            "TERRAIN_OFS_MAX": 0,
            # Turn off input shaping, it makes the plane respond slightly slower which throws the whole test off.
            "RLL2SRV_ACCEL": 0,
            "PTCH2SRV_ACCEL": 0,
        })

        self.install_terrain_handlers_context()
        self.reboot_sitl(check_position=False)

        self.set_parameters({
            "SCR_ENABLE": 1,
            "SIM_SPEEDUP": 20, # need to give some cycles to lua
            "RC7_OPTION": 305,
            "RTL_AUTOLAND": 2,
            "RNGFND1_TYPE": 100,
        })

        self.install_applet_script_context("quadplane_terrain_avoid.lua")
        self.install_script_module_context(self.script_modules_source_path("mavlink_wrappers.lua"), "mavlink_wrappers.lua")
        self.reboot_sitl(check_position=False)
        self.wait_ready_to_arm()

        self.wait_text("Terrain Avoid .* script loaded", regex=True, check_context=True)
        self.set_parameters({
            "TA_CMTC_ENABLE": cmtc_enable,
            "TA_CMTC_RAD": 80,
            "TA_ALT_MAX": 250,
            "WP_LOITER_RAD": 150,
            "RNGFND1_SCALING": 10,
            "RNGFND1_PIN": 0,
            "RNGFND1_MAX": 100,
            "SIM_SONAR_SCALE": 10,
        })

        # This mission triggers an interesting selection of "Pitching", "Quading" and "CMTC" events
        # it's not always consistent, perhaps due to wind, so the tests try to accommodate variances.
        filename = "TopOfTheWorldShort.waypoints"
        self.progress("Flying mission %s" % filename)
        num_wp = self.load_mission(filename)
        self.progress("Mission items %d" % num_wp)

        self.change_mode("AUTO")

        # check that we got terrain data, this test doesn't work if we don't have the correct terrain.
        loc = self.get_location()

        lng_int = int(loc.lng * 1e7)
        lat_int = int(loc.lat * 1e7)

        tstart = self.get_sim_time_cached()
        last_terrain_report_pending = -1
        while True:
            now = self.get_sim_time_cached()
            if now - tstart > 600:
                raise NotAchievedException("Did not get correct terrain report")

            self.mav.mav.terrain_check_send(lat_int, lng_int)

            report = self.assert_receive_message('TERRAIN_REPORT', timeout=60)
            self.progress(self.dump_message_verbose(report))
            if report.spacing != 0:
                break

            # we will keep trying to long as the number of pending
            # tiles is dropping:
            if last_terrain_report_pending == -1:
                last_terrain_report_pending = report.pending
            elif report.pending < last_terrain_report_pending:
                last_terrain_report_pending = report.pending
                tstart = now

            self.delay_sim_time(1, reason="terrain tile download interval")

        self.progress(self.dump_message_verbose(report))
        self.wait_ready_to_arm()

        # TopOfTheWord "ground" is at over 1km altitude
        expected_terrain_height = 1101
        if abs(report.terrain_height - expected_terrain_height) > 1.0:
            raise NotAchievedException("Expected terrain height=%f got=%f" %
                                       (expected_terrain_height, report.terrain_height))

        self.set_rc(7, 1000)
        self.wait_text("TerrAvoid: activated", check_context=True)
        self.set_rc(7, 2000)
        self.wait_text("TerrAvoid: deactivated", check_context=True)
        self.set_rc(7, 1000)
        self.wait_text("TerrAvoid: activated", check_context=True)

        self.progress("TERRAIN_FOLLOW is %f" % self.get_parameter('TERRAIN_FOLLOW'))
        self.progress("TERRAIN_LOOKAHD is %f" % self.get_parameter('TERRAIN_LOOKAHD'))
        self.progress("TERRAIN_OFS_MAX is %f" % self.get_parameter('TERRAIN_OFS_MAX'))
        self.progress("ROLL_LIMIT_DEG is %f" % self.get_parameter('ROLL_LIMIT_DEG'))

    def terrain_avoid_applet_teardown(self):
        '''undo terrain_avoid_applet_setup'''
        # autotest doesn't like this location, so need to move back to Dalby before finishing
        self.customise_SITL_commandline(
            ["--home", "-27.274439,151.290064,343.0,0"]
        )
        self.reboot_sitl()

    @testcase(bucket="QuadPlaneTests1c")
    def TerrainAvoidApplet(self):
        '''Terrain Avoidance with CMTC'''
        self.terrain_avoid_applet_setup(cmtc_enable=1)

        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.wait_text("TerrAvoid: close to home", check_context=True)
        self.wait_waypoint(2, 4, max_dist_to_final_wp_m=100)
        self.wait_text("TerrAvoid: away from home", check_context=True, regex=True)
        self.wait_text("TerrAvoid: CMTC loiter left", check_context=True, regex=True)
        self.progress("CMTC alt #1 is %f" % self.get_altitude(relative=False, timeout=2))
        # wait for CMTC to gain altitude 1170m +-20
        self.wait_altitude(1150, 1190, timeout=60, relative=False, minimum_duration=5)

        self.wait_text("TerrAvoid: CMTC Done|TerrAvoid: Quading overrides CMTC", check_context=True, regex=True, timeout=60)
        self.wait_text("TerrAvoid: CMTC loiter left", check_context=True, regex=True, timeout=60)
        self.progress("CMTC alt #2 is %f" % self.get_altitude(relative=False, timeout=2))
        # wait for CMTC to gain altitude to 1125m +- 55
        self.wait_altitude(1070, 1180, timeout=60, relative=False, minimum_duration=5)
        self.wait_text("TerrAvoid: CMTC Done|TerrAvoid: Quading overrides CMTC", check_context=True, regex=True)

        self.wait_text("TerrAvoid: high terrain detected", check_context=True, regex=True, timeout=60)
        self.wait_text("TerrAvoid: CMTC loiter left", check_context=True, regex=True)
        self.progress("CMTC alt #3 is %f" % self.get_altitude(relative=False, timeout=2))
        # 1020 +- 20
        self.wait_altitude(1000, 1040, timeout=120, relative=False, minimum_duration=5)
        self.wait_text("TerrAvoid: CMTC STOP", check_context=True, regex=True)
        self.wait_text("TerrAvoid: CMTC Done", check_context=True, regex=True)

        # The applet reacts to terrain as it comes into range, so the
        # remaining avoidance events are emergent behaviour; which of CMTC,
        # quading and pitching handles any given piece of terrain depends
        # on exactly where the vehicle is when the terrain is seen.  A CMTC
        # climb which starts a little earlier quite legitimately takes the
        # vehicle over terrain which would otherwise have needed pitching.
        # So fly the rest of the mission and afterwards check that the
        # applet did the work, rather than demanding one exact sequence.
        self.wait_statustext('Land complete', timeout=600)
        self.wait_disarmed(timeout=120) # give quadplane a long time to land

        for (text, count) in [
                ("TerrAvoid: CMTC loiter", 4),
                ("TerrAvoid: CMTC STOP", 4),
                ("TerrAvoid: CMTC Done", 4),
                ("TerrAvoid: Quading started", 1),
                ("TerrAvoid: Quading DONE", 1),
                ("TerrAvoid: terrain Ok", 1),
        ]:
            self.assert_statustext_count_in_collections(text, count)

        self.terrain_avoid_applet_teardown()

    @testcase(bucket="QuadPlaneTests1a")
    def TerrainAvoidAppletPitching(self):
        '''Terrain Avoidance pitching with CMTC disabled'''
        # With CMTC disabled the applet can no longer climb over terrain
        # before reaching it, so pitching (and quading) are the only tools
        # it has left.  That makes the pitching path deterministic in a way
        # the CMTC-enabled flight in TerrainAvoidApplet is not.
        self.terrain_avoid_applet_setup(cmtc_enable=0)

        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.wait_text("TerrAvoid: close to home", check_context=True)

        self.wait_statustext('Land complete', timeout=600)
        self.wait_disarmed(timeout=120) # give quadplane a long time to land

        for (text, count) in [
                ("TerrAvoid: Pitching started", 1),
                ("TerrAvoid: Pitching DONE", 1),
                ("TerrAvoid: terrain Ok", 1),
        ]:
            self.assert_statustext_count_in_collections(text, count)

        # ... and TA_CMTC_ENABLE really did disable CMTC:
        seen = self.statustext_count_in_collections("TerrAvoid: CMTC")
        if seen:
            raise NotAchievedException("CMTC ran with TA_CMTC_ENABLE=0 (%u times)" % seen)

        self.terrain_avoid_applet_teardown()
