'''
Copter autotest: CompassLearnCopyFromEKF

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class CompassLearnCopyFromEKF(TestGroup):
    @testcase(bucket="CopterTests2c")
    def CompassLearnCopyFromEKF(self):
        '''test compass learning whereby we copy learnt offsets from the EKF'''
        # a successful learn makes the firmware save the offsets it found,
        # which the suite has no record of and cannot put back
        self.context_preserve_parameters([
            "COMPASS_OFS_X", "COMPASS_OFS_Y", "COMPASS_OFS_Z",
            "COMPASS_OFS2_X", "COMPASS_OFS2_Y", "COMPASS_OFS2_Z",
            "COMPASS_OFS3_X", "COMPASS_OFS3_Y", "COMPASS_OFS3_Z",
        ])
        self.reboot_sitl()
        self.context_push()
        self.set_parameters({
            "SIM_MAG1_OFS_X": 1100,
        })
        self.assert_prearm_failure("Check mag field", other_prearm_failures_fatal=False)
        self.context_pop()
        self.wait_ready_to_arm()
        new_compass_ofs_x = self.learn_compass_offsets_in_flight()

        self.do_RTL()
        self.assert_parameter_value("COMPASS_OFS_X", new_compass_ofs_x, epsilon=30)
        self.reboot_sitl()
        self.assert_parameter_value("COMPASS_OFS_X", new_compass_ofs_x, epsilon=30)
