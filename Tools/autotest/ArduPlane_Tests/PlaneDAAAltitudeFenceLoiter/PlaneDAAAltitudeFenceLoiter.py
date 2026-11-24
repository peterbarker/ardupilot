'''
Plane autotest: PlaneDAAAltitudeFenceLoiter

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneDAAAltitudeFenceLoiter(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PlaneDAAAltitudeFenceLoiter(self):
        """Altitude-only avoidance must not trap a deliberate mission loiter."""
        self._PlaneDAAAltitudeFenceBand(loiter=True)
