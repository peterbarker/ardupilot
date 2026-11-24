'''
Helicopter autotest: PIDNotches

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class PIDNotches(TestGroup):
    @testcase(bucket="Helicopter")
    def PIDNotches(self):
        """Use dynamic harmonic notch to control motor noise."""
        self.progress("Flying with PID notches")
        self.set_parameters({
            "FILT1_TYPE": 1,
            "FILT2_TYPE": 1,
            "AHRS_EKF_TYPE": 10,
            "INS_LOG_BAT_MASK": 3,
            "INS_LOG_BAT_OPT": 0,
            "INS_GYRO_FILTER": 100, # set the gyro filter high so we can observe behaviour
            "LOG_BITMASK": 65535,
            "LOG_DISARMED": 0,
            "SIM_VIB_FREQ_X": 120,  # roll
            "SIM_VIB_FREQ_Y": 120,  # pitch
            "SIM_VIB_FREQ_Z": 180,  # yaw
            "FILT1_NOTCH_FREQ": 120,
            "FILT2_NOTCH_FREQ": 180,
            "ATC_RAT_RLL_NEF": 1,
            "ATC_RAT_PIT_NEF": 1,
            "ATC_RAT_YAW_NEF": 2,
            "SIM_GYR1_RND": 5,
        })
        self.reboot_sitl()

        self.hover_and_check_matched_frequency_with_fft(5, 20, 350, reverse=True, takeoff=True)
