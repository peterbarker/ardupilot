'''
Copter autotest: RefindGPS

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class RefindGPS(TestGroup):
    @testcase(bucket="CopterTests1e")
    def RefindGPS(self):
        '''Refind the GPS and attempt to RTL rather than continue to land'''
        # https://github.com/ArduPilot/ardupilot/issues/14236
        self.progress("arm the vehicle and takeoff in Guided")
        self.takeoff(50, mode='GUIDED')
        self.progress("fly 50m North (or whatever)")
        old_pos = self.assert_receive_message('GLOBAL_POSITION_INT')
        self.fly_guided_move_global_relative_alt(50, 0, 50)
        self.set_parameter('GPS1_TYPE', 0)
        self.drain_mav()
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 30 and self.mode_is('LAND'):
                self.progress("Bug not reproduced")
                break
            m = self.assert_receive_message('GLOBAL_POSITION_INT', verbose=True)
            pos_delta = self.get_distance_int(old_pos, m)
            self.progress("Distance: %f" % pos_delta)
            if pos_delta < 5:
                raise NotAchievedException("Bug reproduced - returned to near origin")
        self.set_parameter('GPS1_TYPE', 1)
        self.do_RTL()
