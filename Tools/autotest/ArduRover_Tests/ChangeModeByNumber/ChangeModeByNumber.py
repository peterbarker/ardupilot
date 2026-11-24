'''
Rover autotest: ChangeModeByNumber

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class ChangeModeByNumber(TestGroup):
    @testcase(bucket="Rover")
    def ChangeModeByNumber(self):
        '''ensure we can set a mode by number, handy when we don't have a
        pymavlink number for it yet'''
        for (x, want) in (0, 'MANUAL'), (1, 'ACRO'), (3, 3):
            self.change_mode(x)
            self.assert_mode(want)
