'''
Plane autotest: PlaneDAAAltitudeFenceGuidedGoal

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneDAAAltitudeFenceGuidedGoal(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PlaneDAAAltitudeFenceGuidedGoal(self):
        """Clamping must preserve an absolute GUIDED goal and accept a new goal."""
        self._PlaneDAAAltitudeFenceBand(loiter=False, guided=True)
