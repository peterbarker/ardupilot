'''
Copter autotest: CirclePilotRadiusZero

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class CirclePilotRadiusZero(TestGroup):
    @testcase(bucket="CopterTests2a")
    def CirclePilotRadiusZero(self):
        '''Circle pitch stick can reduce the radius to zero'''
        # holding pitch-up shrinks the radius at WP_SPD.  Once the radius
        # target reached zero it used to snap back to CIRCLE_RADIUS_M, so the
        # vehicle could never be brought in to rotate in place.
        self.set_parameters({
            "CIRCLE_RADIUS_M": 10,
            "CIRCLE_RATE": 20,
            "WP_SPD": 2,
        })
        self.takeoff(10, mode='LOITER')
        self.hover()  # circle mode uses throttle input
        self.change_mode('CIRCLE')
        # orbit speed is limited to WP_SPD:
        self.wait_groundspeed(1.5, 2.5, minimum_duration=5, timeout=30)
        # hold pitch-up well past the time needed to reach zero radius:
        self.set_rc(2, 1100)
        self.delay_sim_time(10, reason="radius shrinking to zero")
        self.wait_groundspeed(0, 0.3, minimum_duration=5, timeout=30)
        # the radius must stay at zero once the stick is released:
        self.set_rc(2, 1500)
        self.wait_groundspeed(0, 0.3, minimum_duration=10, timeout=30)
        # pitch-down grows the radius again from zero:
        self.set_rc(2, 1900)
        self.wait_groundspeed(1, 10, timeout=30)
        self.set_rc(2, 1500)
        self.do_RTL()
