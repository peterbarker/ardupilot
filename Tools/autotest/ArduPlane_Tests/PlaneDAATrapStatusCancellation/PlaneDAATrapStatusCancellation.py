'''
Plane autotest: PlaneDAATrapStatusCancellation

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneDAATrapStatusCancellation(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PlaneDAATrapStatusCancellation(self):
        """Pilot takeover and disabling the trap must clear its GCS status."""
        for cancel in ("MANUAL", "LOITER", "disabled"):
            self.start_subtest("trap cancellation: " + cancel)
            self.context_push()
            self._PlaneDAAHungTrapFires(alert_only=False, cancel=cancel)
            self.context_pop()
