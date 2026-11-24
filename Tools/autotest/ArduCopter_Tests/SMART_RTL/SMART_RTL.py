'''
Copter autotest: SMART_RTL

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class SMART_RTL(TestGroup):
    @testcase(bucket="CopterTests1d")
    def SMART_RTL(self):
        '''Check SMART_RTL'''
        self.progress("arm the vehicle and takeoff in Guided")
        self.takeoff(20, mode='GUIDED')
        self.progress("fly around a bit (or whatever)")
        locs = [
            (50, 0, 20),
            (-50, 50, 20),
            (-50, 0, 20),
        ]
        for (lat, lng, alt) in locs:
            self.fly_guided_move_local(lat, lng, alt)

        self.change_mode('SMART_RTL')
        for (lat, lng, alt) in reversed(locs):
            self.wait_distance_to_local_position(
                (lat, lng, -alt),
                0,
                10,
                timeout=60
            )
        self.wait_disarmed()
