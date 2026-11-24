'''
Copter autotest: ParameterChecks

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class ParameterChecks(TestGroup):
    @testcase(bucket="CopterTests1d")
    def ParameterChecks(self):
        '''Test Arming Parameter Checks'''
        self.test_parameter_checks_poscontrol("PSC")
