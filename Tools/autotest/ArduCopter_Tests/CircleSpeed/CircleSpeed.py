'''
Copter autotest: CircleSpeed

AP_FLAKE8_CLEAN
'''

import math

from test_discovery import TestGroup
from test_discovery import testcase


class CircleSpeed(TestGroup):
    # test circle speed - should be constant degrees/second:
    @testcase(bucket="CopterTests1e")
    def CircleSpeed(self):
        '''test the consistent-ground-speed-circling works'''
        self.takeoff(10, mode='LOITER')
        self.hover()  # circle mode uses throttle input
        self.change_mode('CIRCLE')
        tests = [
            (10, 20),
            (20, 10),
        ]
        for (radius_m, rate_degs) in tests:
            self.set_parameters({
                "CIRCLE_RADIUS_M": radius_m,
                "CIRCLE_RATE": rate_degs,
            })
            self.change_mode('LOITER')
            self.change_mode('CIRCLE')
            expected_groundspeed = math.pi * 2 * radius_m * (rate_degs/360)
            self.wait_groundspeed(
                expected_groundspeed-0.1,
                expected_groundspeed+0.1,
                minimum_duration=10,
            )
        self.do_RTL()
