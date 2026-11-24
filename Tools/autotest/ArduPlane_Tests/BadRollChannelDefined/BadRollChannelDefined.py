'''
Plane autotest: BadRollChannelDefined

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class BadRollChannelDefined(TestGroup):
    @testcase(bucket="PlaneTests1a")
    def BadRollChannelDefined(self):
        '''ensure we don't die with a  bad Roll channel defined'''
        self.set_parameter("RCMAP_ROLL", 17)
