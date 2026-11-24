'''
QuadPlane autotest: DoRepositionTerrain

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import AltFrame
from vehicle_test_suite import Location
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase

from quadplane import SITL_START_LOCATION


class DoRepositionTerrain(TestGroup):
    @testcase(bucket="QuadPlaneTests1a")
    def DoRepositionTerrain(self):
        '''test handling of DO_REPOSITION with terrain alt'''
        self.install_terrain_handlers_context()
        self.start_subtest("test reposition with terrain alt")
        self.wait_ready_to_arm()

        dest_alt = 45
        dest = Location(SITL_START_LOCATION.lat,
                        SITL_START_LOCATION.lng,
                        dest_alt,
                        AltFrame.ABOVE_TERRAIN)

        self.set_parameters({
            'Q_GUIDED_MODE': 1,
        })

        self.takeoff(30, mode='GUIDED')

        # fly to higher ground
        self.send_do_reposition(dest)
        self.wait_location(
            dest,
            accuracy=200,
            timeout=600,
            height_accuracy=None,  # dest's altitude is above-terrain; alt checked below
        )
        self.wait_altitude(
            dest_alt-10,
            dest_alt+10,  # use a 10m buffer as the plane needs to go up and down a bit to maintain terrain distance
            minimum_duration=10,
            timeout=50,  # includes time for the terrain altitude to settle
            relative=False,
            altitude_source="TERRAIN_REPORT.current_height"
        )

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

        # two locations 500m apart
        loc1 = self.offset_location_ne(dest, -250, 0)
        loc1.set_alt_m(100, AltFrame.ABOVE_TERRAIN)

        loc2 = self.offset_location_ne(dest, 250, 0)
        loc2.set_alt_m(150, AltFrame.ABOVE_TERRAIN)

        loc3 = loc2.copy()
        loc3.set_alt_m(100, AltFrame.ABOVE_TERRAIN)

        positions = [
            ("Loc1", loc1),
            ("Loc2", loc2),
            ("Loc1", loc1),
            ("Loc3", loc3),
            ("Loc1", loc1),
            ("Loc3", loc3),
            ("Loc1", loc1),
            ("Loc3", loc3),
            ("Loc2", loc2),
            ("Loc3", loc3),
            ("Loc2", loc2),
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
            max_alt_ok = max(start_alt, loc_alt) + 10
            self.progress(f"theight {terrain_height_min:.0f} to {terrain_height_max:.0f} accept {min_alt_ok:.0f}:{max_alt_ok:.0f}") # noqa:E501
            if terrain_height_min < min_alt_ok or terrain_height_max > max_alt_ok:
                raise NotAchievedException(f"terrain range breach {start_alt:.1f} {terrain_height_min:.1f} {terrain_height_max:.1f}")  # noqa:E501

        self.change_mode("QLAND")
        self.mav.motors_disarmed_wait()
