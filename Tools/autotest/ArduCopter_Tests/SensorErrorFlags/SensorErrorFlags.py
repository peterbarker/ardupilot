'''
Copter autotest: SensorErrorFlags

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class SensorErrorFlags(TestGroup):
    @testcase(bucket="CopterTests2d")
    def SensorErrorFlags(self):
        '''Test we get ERR messages when sensors have issues'''
        self.reboot_sitl()

        for (param_names, param_value, expected_subsys, expected_ecode, desc) in [
                (['SIM_BARO_DISABLE', 'SIM_BAR2_DISABLE'], 1, 18, 4, 'unhealthy'),
                (['SIM_BARO_DISABLE', 'SIM_BAR2_DISABLE'], 0, 18, 0, 'healthy'),
                (['SIM_MAG1_FAIL', 'SIM_MAG2_FAIL', 'SIM_MAG3_FAIL'], 1, 3, 4, 'unhealthy'),
                (['SIM_MAG1_FAIL', 'SIM_MAG2_FAIL', 'SIM_MAG3_FAIL'], 0, 3, 0, 'healthy'),
        ]:
            sp = dict()
            for name in param_names:
                sp[name] = param_value
            self.set_parameters(sp)
            # The sensor health change takes time to be detected (up to the
            # baro/compass health timeouts) and the ERR message is flushed to
            # the onboard log asynchronously.  Poll the log until the expected
            # ERR appears rather than reading it once after a fixed delay,
            # which races against detection/flush and makes this test flaky.
            tstart = self.get_sim_time()
            success = False
            while not success:
                if self.get_sim_time_cached() - tstart > 5:
                    raise NotAchievedException("Did not find %s log message" % desc)
                self.delay_sim_time(1, reason="sensor state to update in log")
                mlog = self.dfreader_for_current_onboard_log()
                while True:
                    m = mlog.recv_match(type='ERR')
                    if m is None:
                        break
                    if m.Subsys == expected_subsys and m.ECode == expected_ecode:  # baro / ecode
                        success = True
                        break
