'''
Copter autotest: ThrottleGainBoost

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class ThrottleGainBoost(TestGroup):
    @testcase(bucket="CopterTests1a")
    def ThrottleGainBoost(self):
        """Use PD and Angle P boost for anti-gravity."""
        # basic gyro sample rate test
        self.progress("Flying with Throttle-Gain Boost")

        # magic tridge EKF type that dramatically speeds up the test
        self.set_parameters({
            "AHRS_EKF_TYPE": 10,
            "EK2_ENABLE": 0,
            "EK3_ENABLE": 0,
            "INS_FAST_SAMPLE": 0,
            "LOG_BITMASK": 959,
            "LOG_DISARMED": 0,
            "ATC_THR_G_BOOST": 5.0,
        })

        self.reboot_sitl()

        self.takeoff(10, mode="ALT_HOLD")
        hover_time = 15
        self.progress("Hovering for %u seconds" % hover_time)
        tstart = self.get_sim_time()
        while self.get_sim_time_cached() < tstart + hover_time:
            self.assert_receive_message('ATTITUDE')

        # fly fast forrest!
        self.set_rc_from_map({3: 1900, 2: 1200})
        self.wait_groundspeed(5, 1000)
        self.set_rc_from_map({3: 1500, 2: 1500})

        self.do_RTL()
