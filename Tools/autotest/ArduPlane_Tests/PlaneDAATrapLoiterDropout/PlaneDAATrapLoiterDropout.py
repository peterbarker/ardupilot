'''
Plane autotest: PlaneDAATrapLoiterDropout

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneDAATrapLoiterDropout(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PlaneDAATrapLoiterDropout(self):
        """An owned trap survives position/target dropout, but yields to pilot takeover."""
        for dropout in ("position", "target", "pilot takeover"):
            self.start_subtest("trapped loiter dropout: " + dropout)
            self.context_push()
            self._PlaneDAATrapResumeDuringLoiter(dropout)
            self.context_pop()
