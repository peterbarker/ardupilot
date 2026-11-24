'''
Copter autotest: RangeFinderDriversMaxAlt_AinsteinLRD1

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class RangeFinderDriversMaxAlt_AinsteinLRD1(TestGroup):
    @testcase(bucket="CopterTests1a")
    def RangeFinderDriversMaxAlt_AinsteinLRD1(self) -> None:
        '''test max-height behaviour - Ainstein LRD1 - pre v19.0.0 firmware'''
        self.context_collect('STATUSTEXT')
        self.RangeFinderDriversMaxAlt_FlyDriver(
            name="Ainstein-LR-D1",
            rngfnd_type=42,
            simname='ainsteinlrd1',
            maxalt=500,
            sqalt=495,
            sq_at_sqalt=39,
            expected_statustext='Altitude reading overflow',
        )
        self.wait_text("Rangefinder: Temperature alert", check_context=True)
