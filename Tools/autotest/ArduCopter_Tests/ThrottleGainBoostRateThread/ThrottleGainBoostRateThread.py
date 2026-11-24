'''
Copter autotest: ThrottleGainBoostRateThread

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class ThrottleGainBoostRateThread(TestGroup):
    @testcase(bucket="CopterTests1a")
    def ThrottleGainBoostRateThread(self):
        """Check throttle-gain boost is applied once per loop with the fast rate thread"""
        boost = 0.4
        self.set_parameters({
            "AHRS_EKF_TYPE": 10,
            "EK2_ENABLE": 0,
            "EK3_ENABLE": 0,
            "LOG_BITMASK": 959,
            "LOG_DISARMED": 0,
            "ATC_THR_G_BOOST": boost,
            "FSTRATE_ENABLE": 3,
            "FSTRATE_DIV": 1,
            # with the 1kHz SITL gyro this gives about 5 rate thread runs per main loop
            "SCHED_LOOP_RATE": 200,
        })
        self.reboot_sitl()

        self.takeoff(10, mode="ALT_HOLD")
        self.change_mode('STABILIZE')
        # throttle punches to trigger the boost
        for _ in range(5):
            self.set_rc(3, 1900)
            self.wait_climbrate(2, 50)
            self.set_rc(3, 1300)
            self.wait_climbrate(-50, -1)
        self.set_rc(3, 1500)
        self.do_RTL()

        max_pd = 0
        max_angle_p = 0
        rate_thread_dts = []
        dfreader = self.dfreader_for_current_onboard_log()
        while True:
            m = dfreader.recv_match(type=['ATSC', 'RTDT'])
            if m is None:
                break
            if m.get_type() == 'RTDT':
                rate_thread_dts.append(m.dtAvg)
                continue
            max_pd = max(max_pd, m.PDScX, m.PDScY)
            max_angle_p = max(max_angle_p, m.AngPScX, m.AngPScY)
        self.progress("max PD scale %f, max angle P scale %f" % (max_pd, max_angle_p))

        # without the rate thread running several times per loop the boost cannot compound
        if len(rate_thread_dts) == 0:
            raise NotAchievedException("rate thread did not run")
        mean_dt = sum(rate_thread_dts) / len(rate_thread_dts)
        if mean_dt > 0.5 / 200:
            raise NotAchievedException("rate thread mean dt %f, too slow" % mean_dt)

        expected_pd = 1 + boost
        expected_angle_p = expected_pd * expected_pd
        if abs(max_pd - expected_pd) > 0.001:
            raise NotAchievedException("max PD scale %f, expected %f" % (max_pd, expected_pd))
        if abs(max_angle_p - expected_angle_p) > 0.001:
            raise NotAchievedException("max angle P scale %f, expected %f" % (max_angle_p, expected_angle_p))
