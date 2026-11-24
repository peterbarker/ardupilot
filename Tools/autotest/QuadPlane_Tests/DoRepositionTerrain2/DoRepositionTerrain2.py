'''
QuadPlane autotest: DoRepositionTerrain2

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import AltFrame
from vehicle_test_suite import Location
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class DoRepositionTerrain2(TestGroup):
    @testcase(bucket="QuadPlaneTests1a")
    def DoRepositionTerrain2(self):
        '''test handling of DO_REPOSITION terrain alt2'''
        self.install_terrain_handlers_context()
        self.start_subtest("test reposition terrain alt2")

        takeoff_loc = Location(-35.28243788, 149.00502473, 583.7, AltFrame.ABSOLUTE)
        takeoff_alt_amsl = takeoff_loc.get_alt_m(AltFrame.ABSOLUTE)
        self.customise_SITL_commandline(
            ["--home", f"{takeoff_loc.lat},{takeoff_loc.lng},{takeoff_alt_amsl},0"]
        )
        self.reboot_sitl(check_position=False)
        self.wait_ready_to_arm()

        self.set_parameters({
            'Q_GUIDED_MODE': 1,
        })

        self.takeoff(75, mode='GUIDED', timeout=60)

        # remember the range of heights we go through in the tests
        start_alt = self.assert_receive_message('TERRAIN_REPORT').current_height
        terrain_height_min = start_alt
        terrain_height_max = start_alt

        def terrain_height_range(mav, m):
            if m.get_type() == 'TERRAIN_REPORT':
                nonlocal terrain_height_min, terrain_height_max
                terrain_height_min = min(terrain_height_min, m.current_height)
                terrain_height_max = max(terrain_height_max, m.current_height)

        self.install_message_hook_context(terrain_height_range)

        loc1 = Location(-35.27502040, 148.98635977, 75, AltFrame.ABOVE_TERRAIN)
        loc2 = Location(-35.28505202, 148.98604378, 75, AltFrame.ABOVE_TERRAIN)

        loc3 = Location(-35.27502040, 148.98635977, 120, AltFrame.ABOVE_TERRAIN)
        loc4 = Location(-35.28505202, 148.98604378, 120, AltFrame.ABOVE_TERRAIN)

        loc5 = Location(-35.28505202, 148.98604378, 100, AltFrame.ABOVE_TERRAIN)

        positions = [
            ("Loc1", loc1),
            ("Loc2", loc2),
            ("Loc1", loc1),
            ("Loc2", loc2),
            ("Loc1", loc1),
            ("Loc4", loc4),
            ("Loc3", loc3),
            ("Loc2", loc2),
            ("Loc5", loc5),
            ("Loc1", loc1),
            ("Loc5", loc5),
            ("Loc1", loc1),
        ]
        for (name, loc) in positions:
            start_alt = self.assert_receive_message('TERRAIN_REPORT').current_height
            terrain_height_min = start_alt
            terrain_height_max = start_alt

            loc_alt = loc.get_alt_m(AltFrame.ABOVE_TERRAIN)
            self.progress(f"Flying to {name} at {loc_alt:.1f} from {start_alt:.1f}")
            self.send_do_reposition(loc)

            self.wait_location(
                loc,
                accuracy=10,
                timeout=600,
                height_accuracy=None,  # loc's altitude is above-terrain; alt checked below
            )
            self.wait_altitude(
                loc_alt-5,
                loc_alt+5,
                minimum_duration=10,
                timeout=40,  # includes time for the terrain altitude to settle
                relative=False,
                altitude_source="TERRAIN_REPORT.current_height"
            )

            self.wait_groundspeed(0, 2)
            self.wait_altitude(
                loc_alt-5,
                loc_alt+5,
                minimum_duration=10,
                timeout=30,
                relative=False,
                altitude_source="TERRAIN_REPORT.current_height"
            )
            min_alt_ok = min(start_alt, loc_alt) - 10
            max_alt_ok = max(start_alt, loc_alt) + 25
            self.progress(f"theight {terrain_height_min:.0f} to {terrain_height_max:.0f} accept {min_alt_ok:.0f}:{max_alt_ok:.0f}") # noqa:E501
            if terrain_height_min < min_alt_ok or terrain_height_max > max_alt_ok:
                raise NotAchievedException(f"terrain range breach {start_alt:.1f} {terrain_height_min:.1f} {terrain_height_max:.1f}") # noqa:E501

        self.change_mode("QLAND")
        self.mav.motors_disarmed_wait()
        self.reset_SITL_commandline()
