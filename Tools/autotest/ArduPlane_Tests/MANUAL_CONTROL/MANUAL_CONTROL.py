'''
Plane autotest: MANUAL_CONTROL

AP_FLAKE8_CLEAN
'''

import math

from vehicle_test_suite import AutoTestTimeoutException

from test_discovery import TestGroup
from test_discovery import testcase


class MANUAL_CONTROL(TestGroup):
    @testcase(bucket="PlaneTests1a")
    def MANUAL_CONTROL(self):
        '''test MANUAL_CONTROL mavlink message'''
        self.set_parameter("MAV_GCS_SYSID", self.mav.source_system)

        self.progress("Takeoff")
        self.takeoff(alt=50)

        self.change_mode('FBWA')

        tstart = self.get_sim_time_cached()
        roll_input = -500
        want_roll_degrees = -12
        while True:
            if self.get_sim_time_cached() - tstart > 10:
                raise AutoTestTimeoutException("Did not reach roll")
            self.progress("Sending roll-left")
            self.mav.mav.manual_control_send(
                1, # target system
                32767, # x (pitch)
                roll_input, # y (roll)
                32767, # z (thrust)
                32767, # r (yaw)
                0) # button mask
            m = self.assert_receive_message('ATTITUDE', verbose=True)
            p = math.degrees(m.roll)
            self.progress("roll=%f want<=%f" % (p, want_roll_degrees))
            if p <= want_roll_degrees:
                break
        self.mav.mav.manual_control_send(
            1, # target system
            32767, # x (pitch)
            32767, # y (roll)
            32767, # z (thrust)
            32767, # r (yaw)
            0) # button mask
        self.fly_home_land_and_disarm()
