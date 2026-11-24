'''
QuadPlane autotest: ParameterChecks

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class ParameterChecks(TestGroup):
    @testcase(bucket="QuadPlaneTests1a")
    def ParameterChecks(self):
        '''basic parameter checks'''
        self.test_parameter_checks_poscontrol("Q_P")

        self.context_push()
        self.set_parameters({
            "Q_RTL_MODE": 1,
            "RTL_AUTOLAND": 2,
        })
        self.assert_prearm_failure("unset one of RTL_AUTOLAND or Q_RTL_MODE")
        self.context_pop()
        self.wait_ready_to_arm()
