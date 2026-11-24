'''
Plane autotest: PlaneDAALoiterRespectsPilotTakeover

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneDAALoiterRespectsPilotTakeover(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PlaneDAALoiterRespectsPilotTakeover(self):
        '''A mode change after detection starts must prevent the aircraft loiter.'''
        self._PlaneDAAModeChangeDuringDetection(trap=False)
