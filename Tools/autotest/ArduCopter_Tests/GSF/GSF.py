'''
Copter autotest: GSF

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class GSF(TestGroup):
    @testcase(bucket="CopterTests2c")
    def GSF(self):
        '''test the Gaussian Sum filter'''
        self.context_push()
        self.set_parameter("EK2_ENABLE", 1)
        self.reboot_sitl()
        self.takeoff(20, mode='LOITER')
        self.set_rc(2, 1400)
        self.delay_sim_time(5, reason="GSF to accumulate data")
        self.set_rc(2, 1500)
        self.progress("Path: %s" % self.current_onboard_log_filepath())
        dfreader = self.dfreader_for_current_onboard_log()
        self.do_RTL()
        self.context_pop()
        self.reboot_sitl()

        # ensure log messages present
        want = set(["XKY0", "XKY1", "NKY0", "NKY1"])
        still_want = want
        while len(still_want):
            m = dfreader.recv_match(type=want)
            if m is None:
                raise NotAchievedException("Did not get %s" % want)
            still_want.remove(m.get_type())
