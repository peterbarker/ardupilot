'''
Plane autotest: AutotuneFiltering

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class AutotuneFiltering(TestGroup):
    @testcase(bucket="PlaneTests1a")
    def AutotuneFiltering(self):
        '''Test AutoTune mode with filter updates disabled'''
        self.context_preserve_parameters(self.autotune_saved_gains)
        self.set_parameters({
            "AUTOTUNE_OPTIONS": 3,
            # some filtering is required for autotune to complete
            "RLL_RATE_FLTD": 10,
            "PTCH_RATE_FLTD": 10,
            "RLL_RATE_FLTT": 20,
            "PTCH_RATE_FLTT": 20,
        })
        self.run_autotune()
