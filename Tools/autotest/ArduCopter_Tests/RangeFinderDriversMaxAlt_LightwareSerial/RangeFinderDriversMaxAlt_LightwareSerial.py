'''
Copter autotest: RangeFinderDriversMaxAlt_LightwareSerial

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class RangeFinderDriversMaxAlt_LightwareSerial(TestGroup):
    @testcase(bucket="CopterTests2b")
    def RangeFinderDriversMaxAlt_LightwareSerial(self) -> None:
        '''test max-height behaviour - LightwareSerial'''
        self.RangeFinderDriversMaxAlt_FlyDriver(
            name="LightwareSerial",
            rngfnd_type=8,
            simname='lightwareserial',
            maxalt=100,
            sqalt=95,
            sq_at_sqalt=100,
            expected_statustext=None,
        )
