'''
Plane autotest: PlaneDAALoiterHandbackRespectsPilotTakeover

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneDAALoiterHandbackRespectsPilotTakeover(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PlaneDAALoiterHandbackRespectsPilotTakeover(self):
        '''Expired-cooldown loiter release must preserve an external mode change.'''
        self._PlaneDAALoiterHandbackModeChange(force_stop=False)
