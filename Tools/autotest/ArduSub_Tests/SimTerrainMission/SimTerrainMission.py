'''
Sub autotest: SimTerrainMission

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class SimTerrainMission(TestGroup):
    @testcase(bucket="Sub")
    def SimTerrainMission(self):
        """Mission at a constant height above synthetic sea floor"""

        sea_floor_depth = 50    # Depth of sea floor at location of test
        match_distance = 15     # Desired sub distance from sea floor
        start_altitude = -sea_floor_depth+match_distance
        end_altitude = start_altitude
        validation_delta = 1.5  # Largest allowed distance between sub height and desired height

        # Load the synthetic seafloor, this will push a context and reboot
        self.prepare_synthetic_seafloor_test(sea_floor_depth, match_distance)

        # The synthetic seafloor has an east-west ridge south of the sub.
        # The mission contained in terrain_mission.txt instructs the sub
        # to remain at 15m above the seafloor and travel south over the
        # ridge. Then the sub moves west and travels north over the ridge.
        filename = "terrain_mission.txt"
        self.load_mission(filename)

        # Dive to match_distance off the bottom in preparation for the mission
        self.dive(start_altitude)
        self.change_mode('AUTO')
        self.watch_true_distance_maintained(match_distance, delta=validation_delta, timeout=500.0, final_waypoint=4)

        # The mission ends at end_altitude. Do a check to ensure that the sub is at this altitude.
        self.wait_altitude(altitude_min=end_altitude-validation_delta/2, altitude_max=end_altitude+validation_delta/2,
                           relative=False, timeout=1)

        # Disarm before context pop
        self.disarm_vehicle()
