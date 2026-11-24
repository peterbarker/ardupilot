'''
QuadPlane autotest: FwdThrInVTOL

AP_FLAKE8_CLEAN
'''

import math

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class FwdThrInVTOL(TestGroup):
    @testcase(bucket="QuadPlaneTests1a")
    def FwdThrInVTOL(self):
        '''test use of fwd motor throttle into wind'''
        self.set_parameters({"SIM_WIND_SPD": 25, # need very strong wind for this test
                             "SIM_WIND_DIR": 360,
                             "Q_WVANE_ENABLE": 1,
                             "Q_WVANE_GAIN": 1,
                             "STICK_MIXING": 0,
                             "Q_FWD_THR_USE": 2})

        self.takeoff(10, mode="QLOITER")
        self.set_rc(2, 1000)
        self.delay_sim_time(10, reason="forward movement into wind")
        # Check that it is using some forward throttle
        fwd_thr_pwm = self.get_servo_channel_value(3)
        if fwd_thr_pwm < 1150 :
            raise NotAchievedException("fwd motor pwm command low, want >= 1150 got %f" % (fwd_thr_pwm))
        # check that pitch is on limit
        m = self.assert_receive_message('ATTITUDE')
        pitch = math.degrees(m.pitch)
        if abs(pitch + 3.0) > 0.5 :
            raise NotAchievedException("pitch should be -3.0 +- 0.5 deg, got %f" % (pitch))
        self.set_rc(2, 1500)
        self.delay_sim_time(5, reason="position to stabilise")
        loc1 = self.get_location()
        self.set_parameter("SIM_ENGINE_FAIL", 1 << 2) # simulate a complete loss of forward motor thrust
        self.delay_sim_time(20, reason="engine failure effect")
        self.change_mode('QLAND')
        self.wait_disarmed(timeout=60)
        loc2 = self.get_location()
        position_drift = self.get_distance(loc1, loc2)
        if position_drift > 5.0 :
            raise NotAchievedException("position drift high, want < 5.0 m got %f m" % (position_drift))
