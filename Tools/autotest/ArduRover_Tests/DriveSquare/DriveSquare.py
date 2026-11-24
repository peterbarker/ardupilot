'''
Rover autotest: DriveSquare

AP_FLAKE8_CLEAN
'''

import os

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase

from rover import testdir


class DriveSquare(TestGroup):
    ##########################################################
    #   TESTS DRIVE
    ##########################################################
    # Drive a square in manual mode
    @testcase(bucket="Rover")
    def DriveSquare(self, side=50):
        """Learn/Drive Square with Ch7 option"""

        self.progress("TEST SQUARE")
        self.set_parameters({
            "RC7_OPTION": 7,
            "RC9_OPTION": 58,
        })

        self.change_mode('MANUAL')

        self.wait_ready_to_arm()
        self.arm_vehicle()

        self.clear_wp(9)

        # first aim north
        self.progress("\nTurn right towards north")
        self.reach_heading_manual(10)
        # save bottom left corner of box as home AND waypoint
        self.progress("Save HOME")
        self.save_wp()

        self.progress("Save WP")
        self.save_wp()

        # pitch forward to fly north
        self.progress("\nGoing north %u meters" % side)
        self.reach_distance_manual(side)
        # save top left corner of square as waypoint
        self.progress("Save WP")
        self.save_wp()

        # roll right to fly east
        self.progress("\nGoing east %u meters" % side)
        self.reach_heading_manual(100)
        self.reach_distance_manual(side)
        # save top right corner of square as waypoint
        self.progress("Save WP")
        self.save_wp()

        # pitch back to fly south
        self.progress("\nGoing south %u meters" % side)
        self.reach_heading_manual(190)
        self.reach_distance_manual(side)
        # save bottom right corner of square as waypoint
        self.progress("Save WP")
        self.save_wp()

        # roll left to fly west
        self.progress("\nGoing west %u meters" % side)
        self.reach_heading_manual(280)
        self.reach_distance_manual(side)
        # save bottom left corner of square (should be near home) as waypoint
        self.progress("Save WP")
        self.save_wp()

        self.progress("Checking number of saved waypoints")
        mavproxy = self.start_mavproxy()
        num_wp = self.save_mission_to_file_using_mavproxy(
            mavproxy,
            os.path.join(testdir, "ch7_mission.txt"))
        self.stop_mavproxy(mavproxy)
        expected = 7 # home + 6 toggled in
        if num_wp != expected:
            raise NotAchievedException("Did not get %u waypoints; got %u" %
                                       (expected, num_wp))

        # TODO: actually drive the mission

        self.clear_wp(9)

        self.disarm_vehicle()
