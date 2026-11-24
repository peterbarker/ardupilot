'''
Copter autotest: EK3YawAlignEachTakeoff

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class EK3YawAlignEachTakeoff(TestGroup):
    @testcase(bucket="CopterTests2a")
    def EK3YawAlignEachTakeoff(self):
        '''EKF3 redoes the in-flight yaw alignment on each takeoff, including after landing while armed'''
        alignment_text = "EKF3 IMU0 MAG0 in-flight yaw alignment complete"
        self.context_collect('STATUSTEXT')
        self.takeoff(altitude_min=10, mode='ALT_HOLD')
        self.wait_statustext(alignment_text, check_context=True, timeout=10)
        # a low but not minimum throttle lands the vehicle without disarming it
        self.set_rc(3, 1300)
        self.wait_altitude(-1, 0.3, relative=True, timeout=120)
        self.delay_sim_time(5, reason="let the land detector trigger")
        # take off again without disarming and expect the alignment again
        self.context_clear_collection('STATUSTEXT')
        self.set_rc(3, 1700)
        self.wait_statustext(alignment_text, check_context=True, timeout=30)
        self.set_rc(3, 1500)
        # return home so the vehicle ends the test where it started
        self.do_RTL()
