'''
Plane autotest: PlaneDAALoiterForcedStopRespectsPilotTakeover

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneDAALoiterForcedStopRespectsPilotTakeover(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PlaneDAALoiterForcedStopRespectsPilotTakeover(self):
        '''Forced loiter cleanup must preserve an external mode change.'''
        self._PlaneDAALoiterHandbackModeChange(force_stop=True)
