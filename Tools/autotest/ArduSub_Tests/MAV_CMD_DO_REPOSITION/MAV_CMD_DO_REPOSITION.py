'''
Sub autotest: MAV_CMD_DO_REPOSITION

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import AltFrame

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_DO_REPOSITION(TestGroup):
    @testcase(bucket="Sub")
    def MAV_CMD_DO_REPOSITION(self):
        """Move vehicle using MAV_CMD_DO_REPOSITION"""
        # Dive so that rangefinder is in range, required for MAV_FRAME_GLOBAL_TERRAIN_ALT
        start_altitude = -25
        self.dive(start_altitude)
        self.change_mode('GUIDED')

        loc = self.get_location()

        # Reposition, alt relative to surface
        loc = self.offset_location_ne(loc, 10, 10)
        loc.set_alt_m(start_altitude, AltFrame.ABOVE_HOME)
        self.send_do_reposition(loc)
        self.wait_location(loc, timeout=120)

        # Reposition, alt relative to seafloor.  The SITL seafloor is
        # ~50m deep here, so holding 25m above it is ~-25m AMSL.
        seafloor_depth = 50
        alt_above_seafloor = 25
        target_terrain = self.offset_location_ne(loc, 10, 10)
        target_terrain.set_alt_m(alt_above_seafloor, AltFrame.ABOVE_TERRAIN)
        target_global = self.offset_location_ne(loc, 10, 10)
        target_global.set_alt_m(alt_above_seafloor - seafloor_depth, AltFrame.ABSOLUTE)
        self.send_do_reposition(target_terrain)
        self.wait_location(target_global, timeout=120, height_accuracy=5)

        self.disarm_vehicle()
