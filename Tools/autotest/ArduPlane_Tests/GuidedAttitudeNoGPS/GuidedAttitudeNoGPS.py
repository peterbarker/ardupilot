'''
Plane autotest: GuidedAttitudeNoGPS

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class GuidedAttitudeNoGPS(TestGroup):
    @testcase(bucket="PlaneTests1c")
    def GuidedAttitudeNoGPS(self):
        '''test that guided-attitude still works with no GPS'''
        self.takeoff(30)
        self.change_mode('GUIDED')
        self.context_push()
        self.set_parameter('SIM_GPS1_ENABLE', 0)
        self.delay_sim_time(30, reason="GPS disable to take effect")
        self.set_attitude_target()
        self.context_pop()
        self.fly_home_land_and_disarm()
