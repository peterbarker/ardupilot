'''
BalanceBot autotest: DO_SET_MODE

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class DO_SET_MODE(TestGroup):
    @testcase(bucket="BalanceBot")
    def DO_SET_MODE(self):
        '''Set mode via MAV_COMMAND_DO_SET_MODE'''
        self.do_set_mode_via_command_long("HOLD")
        self.do_set_mode_via_command_long("MANUAL")
