'''
Sub autotest: SimTerrainSurftrak

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase

from ardusub import Joystick


class SimTerrainSurftrak(TestGroup):
    @testcase(bucket="Sub")
    def SimTerrainSurftrak(self):
        """Move at a constant height above synthetic sea floor"""

        sea_floor_depth = 50    # Depth of sea floor at location of test
        match_distance = 15     # Desired sub distance from sea floor
        start_altitude = -sea_floor_depth+match_distance
        end_altitude = start_altitude
        validation_delta = 1.5  # Largest allowed distance between sub height and desired height

        # Load the synthetic seafloor, this will push a context and reboot
        self.prepare_synthetic_seafloor_test(sea_floor_depth, match_distance)

        # Dive to match_distance off the bottom in preparation for the mission
        self.dive(start_altitude)

        # Turn on surftrak and move around
        self.change_mode(21)

        # Go south over the ridge.
        self.reach_heading_manual(180)
        self.set_rc(Joystick.Forward, 1650)
        self.watch_true_distance_maintained(match_distance, delta=validation_delta, timeout=60)
        self.set_rc(Joystick.Forward, 1500)

        # Shift west a bit
        self.reach_heading_manual(270)
        self.set_rc(Joystick.Forward, 1650)
        self.watch_true_distance_maintained(match_distance, delta=validation_delta, timeout=5)
        self.set_rc(Joystick.Forward, 1500)

        # Go south past the ridge
        self.reach_heading_manual(180)
        self.set_rc(Joystick.Forward, 1650)
        self.watch_true_distance_maintained(match_distance, delta=validation_delta, timeout=60)

        # The mission ends at end_altitude. Do a check to ensure that the sub is at this altitude
        self.wait_altitude(altitude_min=end_altitude-validation_delta/2, altitude_max=end_altitude+validation_delta/2,
                           relative=False, timeout=1)

        self.set_rc(Joystick.Forward, 1500)

        # Disarm before context pop
        self.disarm_vehicle()
