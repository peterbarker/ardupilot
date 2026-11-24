'''
Plane autotest: PlaneDAAHungAlertClears

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneDAAHungAlertClears(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PlaneDAAHungAlertClears(self):
        """An ordinary HUNG alert must still announce clearing when the mission advances."""
        self._PlaneDAAHungTrapFires(alert_only=True)
