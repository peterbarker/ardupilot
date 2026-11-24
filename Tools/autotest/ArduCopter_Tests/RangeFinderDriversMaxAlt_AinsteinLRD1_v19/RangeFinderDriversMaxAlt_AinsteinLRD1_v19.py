'''
Copter autotest: RangeFinderDriversMaxAlt_AinsteinLRD1_v19

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class RangeFinderDriversMaxAlt_AinsteinLRD1_v19(TestGroup):
    @testcase(bucket="CopterTests1a")
    def RangeFinderDriversMaxAlt_AinsteinLRD1_v19(self) -> None:
        '''test max-height behaviour - Ainstein LRD1 - v19.0.0 firmware'''
        self.context_collect('STATUSTEXT')
        self.RangeFinderDriversMaxAlt_FlyDriver(
            name="Ainstein-LR-D1-v19",
            rngfnd_type=42,
            simname='ainsteinlrd1_v19',
            maxalt=500,
            sqalt=495,
            sq_at_sqalt=39,
        )
        self.wait_text("Rangefinder: MCU Temperature alert", check_context=True)
