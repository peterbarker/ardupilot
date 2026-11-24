'''
Plane autotest: PlaneDAAAltitudeFenceBand

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneDAAAltitudeFenceBand(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PlaneDAAAltitudeFenceBand(self):
        """Overlapping DAA altitude buffers must not command outside the real fence band."""
        self._PlaneDAAAltitudeFenceBand(loiter=False)
