'''
Copter autotest: MANUAL_CONTROL

AP_FLAKE8_CLEAN
'''

import math

from vehicle_test_suite import AutoTestTimeoutException

from test_discovery import TestGroup
from test_discovery import testcase


class MANUAL_CONTROL(TestGroup):
    @testcase(bucket="CopterTests2b")
    def MANUAL_CONTROL(self):
        '''test MANUAL_CONTROL mavlink message'''
        self.set_parameter("MAV_GCS_SYSID", self.mav.source_system)

        self.takeoff(10, mode='GUIDED')
        self.hover()
        self.change_mode('STABILIZE')

        tstart = self.get_sim_time_cached()
        want_pitch_degrees = -12
        while True:
            if self.get_sim_time_cached() - tstart > 10:
                raise AutoTestTimeoutException("Did not reach pitch")
            self.progress("Sending pitch-forward")
            self.mav.mav.manual_control_send(
                1, # target system
                500, # x (pitch)
                32767, # y (roll)
                32767, # z (thrust)
                32767, # r (yaw)
                0) # button mask
            m = self.assert_receive_message('ATTITUDE', verbose=True)
            p = math.degrees(m.pitch)
            self.progress("pitch=%f want<=%f" % (p, want_pitch_degrees))
            if p <= want_pitch_degrees:
                break
        self.mav.mav.manual_control_send(
            1, # target system
            32767, # x (pitch)
            32767, # y (roll)
            32767, # z (thrust)
            32767, # r (yaw)
            0) # button mask
        self.do_RTL()
