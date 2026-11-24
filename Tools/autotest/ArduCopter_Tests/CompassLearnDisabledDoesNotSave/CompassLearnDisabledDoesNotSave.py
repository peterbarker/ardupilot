'''
Copter autotest: CompassLearnDisabledDoesNotSave

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class CompassLearnDisabledDoesNotSave(TestGroup):
    @testcase(bucket="CopterTests2d")
    def CompassLearnDisabledDoesNotSave(self):
        '''check EKF-learned compass offsets are not saved if COMPASS_LEARN is turned off before landing'''
        self.context_preserve_parameters([
            "COMPASS_OFS_X", "COMPASS_OFS_Y", "COMPASS_OFS_Z",
            "COMPASS_OFS2_X", "COMPASS_OFS2_Y", "COMPASS_OFS2_Z",
            "COMPASS_OFS3_X", "COMPASS_OFS3_Y", "COMPASS_OFS3_Z",
        ])
        self.reboot_sitl()
        self.wait_ready_to_arm()
        self.learn_compass_offsets_in_flight()
        self.set_parameter("COMPASS_LEARN", 0)
        self.do_RTL()
        self.assert_parameter_value("COMPASS_OFS_X", 20, epsilon=30)
