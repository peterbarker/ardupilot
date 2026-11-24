'''
Copter autotest: CircleManualControlEntryRight

AP_FLAKE8_CLEAN
'''

import math

from test_discovery import TestGroup
from test_discovery import testcase


class CircleManualControlEntryRight(TestGroup):
    @testcase(bucket="CopterTests2a")
    def CircleManualControlEntryRight(self):
        '''Circle orbits at the configured rate when entered holding roll stick right'''
        # Holding the roll stick with the CIRCLE_RATE sign on entry used to
        # leave the rate crawling up from zero instead of starting at the
        # configured rate.  The orbit must reach the configured rate.
        self.set_parameters({
            "CIRCLE_RADIUS_M": 10,
            "CIRCLE_RATE": 20,  # +ve, clockwise
        })
        expected_groundspeed = math.pi * 2 * 10 * (20 / 360.0)  # ~3.49m/s
        self.takeoff(10, mode='LOITER')
        self.hover()  # circle mode uses throttle input
        self.set_rc(1, 1600)  # held right, with the CIRCLE_RATE sign
        self.change_mode('CIRCLE')
        self.wait_groundspeed(
            expected_groundspeed - 0.5,
            expected_groundspeed + 0.5,
            minimum_duration=5,
            timeout=30,
        )
        self.set_rc(1, 1500)
        self.do_RTL()
