'''
BalanceBot autotest: DriveRTL

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase

from balancebot import AutoTestBalanceBot


class DriveRTL(TestGroup):
    @testcase(bucket="BalanceBot")
    def DriveRTL(self):
        '''Drive an RTL Mission'''
        # if we Hold then the balancebot continues to wander
        # indefinitely at ~1m/s, hence we set to Acro
        self.set_parameter("MIS_DONE_BEHAVE", 2)
        super(AutoTestBalanceBot, self).DriveRTL()
