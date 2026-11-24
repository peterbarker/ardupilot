'''
QuadPlane autotest: RTLPauseTime

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class RTLPauseTime(TestGroup):
    @testcase(bucket="QuadPlaneTests1a")
    def RTLPauseTime(self):
        '''test Q_RTL_PAUSE_TIME - pause above landing point before descent'''

        def fly_to_fw_and_away():
            '''take off in VTOL, transition to FW and fly 400m+ from home'''
            self.zero_throttle()
            self.takeoff(15, 'QHOVER')
            self.change_mode("FBWA")
            self.set_rc(3, 1900)
            self.wait_distance_to_home(400, 1000, timeout=60)
            self.set_rc(3, 1500)

        # Sub-test 1: Q_RTL_PAUSE_TIME=0 should skip the loiter phase entirely
        self.progress("Testing Q_RTL_PAUSE_TIME=0 - no loiter expected")
        self.context_push()
        self.set_parameter('Q_RTL_PAUSE_TIME', 0)
        fly_to_fw_and_away()
        self.context_collect('STATUSTEXT')
        self.change_mode('QRTL')
        self.wait_statustext('Land descend started', timeout=120)
        if self.statustext_in_collections('Land pause started'):
            raise NotAchievedException(
                "Got unexpected 'Land pause started' with Q_RTL_PAUSE_TIME=0")
        self.wait_disarmed(timeout=120)
        self.context_pop()

        # Sub-test 2: Q_RTL_PAUSE_TIME=5 should delay descent by ~5 seconds
        loiter_time_s = 5
        tolerance_s = 0.5
        self.progress("Testing Q_RTL_PAUSE_TIME=%d - loiter expected" % loiter_time_s)
        self.context_push()
        self.set_parameter('Q_RTL_PAUSE_TIME', loiter_time_s)
        fly_to_fw_and_away()
        self.change_mode('QRTL')
        self.wait_statustext('Land pause started', timeout=120)
        t_loiter_start = self.get_sim_time_cached()
        self.wait_statustext('Land descend started', timeout=60)
        t_descend_start = self.get_sim_time_cached()
        delta = t_descend_start - t_loiter_start
        self.progress("Loiter lasted %.1fs (expected %.1fs)" % (delta, loiter_time_s))
        if abs(delta - loiter_time_s) > tolerance_s:
            raise NotAchievedException(
                "Loiter duration incorrect: got %.1fs expected %.1fs" %
                (delta, loiter_time_s))
        self.wait_disarmed(timeout=120)
        self.context_pop()
