'''
Rover autotest: RCDuplicateOptionsExist

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class RCDuplicateOptionsExist(TestGroup):
    @testcase(bucket="Rover")
    def RCDuplicateOptionsExist(self):
        '''ensure duplicate RC option detection works'''
        self.wait_ready_to_arm()
        self.set_parameters({
            "RC6_OPTION": 118,
            "RC7_OPTION": 118,
        })
        self.wait_not_ready_to_arm()
        self.assert_prearm_failure("Duplicate Aux Switch Options")
        self.assert_arm_failure("Duplicate Aux Switch Options")
