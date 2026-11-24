'''
Copter autotest: ModeAltHold

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class ModeAltHold(TestGroup):
    @testcase(bucket="CopterTests1e")
    def ModeAltHold(self):
        '''Test AltHold Mode'''
        self.takeoff(10, mode="ALT_HOLD")
        self.wait_altitude(
            altitude_min=9,
            altitude_max=11,
            relative=True,
            minimum_duration=5,
            timeout=6,
        )
        # feed in full elevator and aileron input and make sure we
        # retain altitude:
        self.set_rc_from_map({
            1: 1000,
            2: 1000,
        })
        self.wait_altitude(
            altitude_min=9,
            altitude_max=11,
            relative=True,
            minimum_duration=5,
            timeout=6,
        )
        self.set_rc_from_map({
            1: 1500,
            2: 1500,
        })
        self.do_RTL()
