'''
Plane autotest: CompassLearnInFlight

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class CompassLearnInFlight(TestGroup):
    @testcase(bucket="PlaneTests1a")
    def CompassLearnInFlight(self):
        '''check we can learn compass offsets in flight'''
        # a successful learn makes the firmware save the offsets it found,
        # which the suite has no record of and cannot put back
        self.context_preserve_parameters([
            "COMPASS_OFS_X", "COMPASS_OFS_Y", "COMPASS_OFS_Z",
            "COMPASS_OFS2_X", "COMPASS_OFS2_Y", "COMPASS_OFS2_Z",
            "COMPASS_OFS3_X", "COMPASS_OFS3_Y", "COMPASS_OFS3_Z",
        ])
        # learning completes only when the EKF's GSF yaw estimator is
        # converged.  Flying done by earlier tests in the same boot can
        # leave the EKF in a state where the GSF hovers above the
        # convergence threshold for the length of this test - so reboot
        # to run from a known state
        self.reboot_sitl()
        self.context_push()
        self.set_parameters({
            "COMPASS_OFS_X": 1100,
        })
        self.assert_prearm_failure("Check mag field", other_prearm_failures_fatal=False)
        self.context_pop()
        self.wait_ready_to_arm()
        self.takeoff(30, mode='TAKEOFF')
        self.assert_parameter_value("COMPASS_OFS_X", 20, epsilon=30)
        # fly straight and level for a bit to let GSF converge for accurate learning
        self.change_mode("FBWB") # not "CRUISE" to avoid heading track with bad compass
        self.wait_distance(200, accuracy=20)
        old_compass_ofs_x = self.get_parameter('COMPASS_OFS_X')
        self.set_parameters({
            "COMPASS_OFS_X": 1100,
        })
        # the autopilot zeroes COMPASS_LEARN itself when learning
        # completes, so a verified parameter-set could race with that
        # and re-trigger learning; send it unverified, but tracked so
        # a test failure reverts it - a leaked COMPASS_LEARN=3 stops
        # the EKFs using the compass, breaking all subsequent tests
        self.send_set_parameter("COMPASS_LEARN", 3, add_to_context=True)  # 3 is in-flight learning
        self.wait_parameter_value("COMPASS_LEARN", 0)
        self.assert_parameter_value("COMPASS_OFS_X", old_compass_ofs_x, epsilon=30)
        self.fly_home_land_and_disarm()
        self.reboot_sitl()
        self.assert_parameter_value("COMPASS_OFS_X", old_compass_ofs_x, epsilon=30)
