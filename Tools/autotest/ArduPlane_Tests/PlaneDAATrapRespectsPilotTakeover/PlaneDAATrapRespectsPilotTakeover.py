'''
Plane autotest: PlaneDAATrapRespectsPilotTakeover

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneDAATrapRespectsPilotTakeover(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PlaneDAATrapRespectsPilotTakeover(self):
        '''A mode change after detection starts must prevent a new trapped failsafe.'''
        self._PlaneDAAModeChangeDuringDetection(trap=True)
