'''
Copter autotest: CircleManualControlEntryLeft

AP_FLAKE8_CLEAN
'''

import math

from test_discovery import TestGroup
from test_discovery import testcase


class CircleManualControlEntryLeft(TestGroup):
    @testcase(bucket="CopterTests1d")
    def CircleManualControlEntryLeft(self):
        '''Circle orbits at the configured rate when entered holding roll stick left'''
        # CIRCLE_OPTIONS defaults to 1 (manual control enabled); the pilot
        # roll stick trims the orbit rate.  Holding the stick against the
        # CIRCLE_RATE sign on entry used to pin the rate at zero so the
        # vehicle never orbited at all.  The orbit must still reach the
        # configured rate, and the pilot can then trim it down to a stop.
        self.set_parameters({
            "CIRCLE_RADIUS_M": 10,
            "CIRCLE_RATE": 20,  # +ve, clockwise
        })
        expected_groundspeed = math.pi * 2 * 10 * (20 / 360.0)  # ~3.49m/s
        self.takeoff(10, mode='LOITER')
        self.hover()  # circle mode uses throttle input
        self.set_rc(1, 1400)  # held left, against the CIRCLE_RATE sign
        self.change_mode('CIRCLE')
        self.wait_groundspeed(
            expected_groundspeed - 0.5,
            expected_groundspeed + 0.5,
            minimum_duration=5,
            timeout=30,
        )
        # the pilot can still trim the orbit rate down to a stop:
        self.set_rc(1, 1100)
        self.wait_groundspeed(0, 0.5, minimum_duration=2, timeout=60)
        self.set_rc(1, 1500)
        self.do_RTL()
