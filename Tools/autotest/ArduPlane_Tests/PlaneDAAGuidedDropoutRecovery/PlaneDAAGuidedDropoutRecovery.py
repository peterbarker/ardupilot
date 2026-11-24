'''
Plane autotest: PlaneDAAGuidedDropoutRecovery

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneDAAGuidedDropoutRecovery(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PlaneDAAGuidedDropoutRecovery(self):
        """Rejected dropout restores retain the goal, but yield to newer instructions."""
        for dropout in ("position", "target", "new goal", "mode change"):
            self.start_subtest("GUIDED dropout: " + dropout)
            self._PlaneDAAAltitudeFenceBand(loiter=False, guided=True, dropout=dropout)
