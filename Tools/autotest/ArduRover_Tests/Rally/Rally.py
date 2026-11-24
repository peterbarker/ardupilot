'''
Rover autotest: Rally

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class Rally(TestGroup):
    @testcase(bucket="Rover")
    def Rally(self):
        '''Test Rally Points'''
        self.load_rally_using_mavproxy("rover-test-rally.txt")
        self.assert_parameter_value('RALLY_TOTAL', 2)

        # the file's rally points are fixed coordinates chosen relative
        # to the SITL startup location, but RTL returns to whichever of
        # home and the rally points is closest - and home is wherever
        # the vehicle happened to be.  Re-place them relative to us so
        # the rally point is the closer of the two once we have driven
        # away, whatever a previous test did with the vehicle:
        here = self.get_location()
        rally_locs = [
            self.offset_location_ne(here, 19.8, 33.0),
            self.offset_location_ne(here, 99.1, -114.7),
        ]
        self.upload_rally_points_from_locations(rally_locs)

        self.wait_ready_to_arm()
        self.arm_vehicle()

        # calculate early to avoid round-trips while vehicle is moving:
        accuracy = self.get_parameter("WP_RADIUS")

        self.reach_heading_manual(10)
        self.reach_distance_manual(50)

        self.change_mode("RTL")

        loc = rally_locs[0]

        self.wait_location(loc, accuracy=accuracy, minimum_duration=10, timeout=45)
        self.disarm_vehicle()
