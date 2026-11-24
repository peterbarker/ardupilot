'''
Copter autotest: StaticNotches

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class StaticNotches(TestGroup):
    @testcase(bucket="CopterTests2b")
    def StaticNotches(self):
        """Use static harmonic notch to control motor noise."""
        self.progress("Flying with Static notches")
        self.set_parameters({
            "AHRS_EKF_TYPE": 10,
            "INS_LOG_BAT_MASK": 3,
            "INS_LOG_BAT_OPT": 4,
            "INS_GYRO_FILTER": 100, # set the gyro filter high so we can observe behaviour
            "LOG_BITMASK": 65535,
            "LOG_DISARMED": 0,
            "SIM_VIB_FREQ_X": 120,  # roll
            "SIM_VIB_FREQ_Y": 120,  # pitch
            "SIM_VIB_FREQ_Z": 120,  # yaw
            "SIM_VIB_MOT_MULT": 0,
            "SIM_GYR1_RND": 5,
            "INS_HNTCH_ENABLE": 1,
            "INS_HNTCH_FREQ": 120,
            "INS_HNTCH_REF": 1.0,
            "INS_HNTCH_HMNCS": 3, # first and second harmonic
            "INS_HNTCH_ATT": 50,
            "INS_HNTCH_BW": 40,
            "INS_HNTCH_MODE": 0, # static notch
        })
        self.reboot_sitl()

        self.hover_and_check_matched_frequency_with_fft(dblevel=-15, minhz=20, maxhz=350, reverse=True, instance=2)
