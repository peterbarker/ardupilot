'''
Copter autotest: AvoidanceAltFence

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class AvoidanceAltFence(TestGroup):
    @testcase(bucket="CopterTests2d")
    def AvoidanceAltFence(self):
        '''Test fence avoidance at minimum and maximum altitude'''
        self.context_push()

        self.set_parameters({
            "FENCE_ENABLE": 1,
            "FENCE_TYPE": 9,   # min and max alt fence
            "FENCE_ALT_MIN": 10,
            "FENCE_ALT_MAX": 30,
        })

        self.change_mode('LOITER')
        self.wait_ekf_happy()

        tstart = self.get_sim_time()
        self.takeoff(15, mode='LOITER')
        self.progress("Increasing throttle, vehicle should stay below 30m")
        self.set_rc(3, 1920)

        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 20:
                break
            alt = self.get_altitude(relative=True)
            self.progress("Altitude %s" % alt)
            if alt > 30:
                raise NotAchievedException("Breached maximum altitude (%s)" % (str(alt),))

        self.progress("Decreasing, vehicle should stay above 10m")
        self.set_rc(3, 1080)
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 20:
                break
            alt = self.get_altitude(relative=True)
            self.progress("Altitude %s" % alt)
            if alt < 10:
                raise NotAchievedException("Breached minimum altitude (%s)" % (str(alt),))
        self.context_pop()

        self.do_RTL()
