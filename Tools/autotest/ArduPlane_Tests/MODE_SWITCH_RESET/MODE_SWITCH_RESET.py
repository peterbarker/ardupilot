'''
Plane autotest: MODE_SWITCH_RESET

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class MODE_SWITCH_RESET(TestGroup):
    @testcase(bucket="PlaneTests1c")
    def MODE_SWITCH_RESET(self):
        '''test the MODE_SWITCH_RESET auxiliary function'''
        self.set_parameters({
            "RC9_OPTION": 96,
        })

        self.progress("Using RC to change modes")
        self.set_rc(8, 1500)
        self.wait_mode('FBWA')

        self.progress("Killing RC to engage RC failsafe")
        self.set_parameter('SIM_RC_FAIL', 1)
        self.wait_mode('RTL')

        self.progress("Reinstating RC")
        self.set_parameter('SIM_RC_FAIL', 0)

        self.progress("Ensuring we don't automatically revert mode")
        self.delay_sim_time(2, reason="mode to not automatically revert")
        self.assert_mode_is('RTL')

        self.progress("Ensuring MODE_SWITCH_RESET switch resets to pre-failsafe mode")
        self.set_rc(9, 2000)
        self.wait_mode('FBWA')
