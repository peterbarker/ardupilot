'''
Rover autotest: DriveMaxRCIN

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class DriveMaxRCIN(TestGroup):
    @testcase(bucket="Rover")
    def DriveMaxRCIN(self, timeout=30):
        """Drive rover at max RC inputs"""
        self.progress("Testing max RC inputs")
        self.change_mode("MANUAL")

        self.wait_ready_to_arm()
        self.arm_vehicle()

        self.set_rc_from_map({3: 2000, 1: 1000})

        tstart = self.get_sim_time()
        while self.get_sim_time_cached() - tstart < timeout:
            m = self.assert_receive_message('VFR_HUD')
            self.progress("Current speed: %f" % m.groundspeed)

        # centre the sticks and let the rover coast to a stop before we
        # finish.  Disarming stops the motors but not the vehicle, and
        # the next test is entitled to start from rest.
        self.set_rc_from_map({3: 1500, 1: 1500})
        self.disarm_vehicle()
        self.wait_groundspeed(0, 0.2, minimum_duration=1)
