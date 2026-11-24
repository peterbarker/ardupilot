'''
Copter autotest: GyroFFTMotorNoiseCheck

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class GyroFFTMotorNoiseCheck(TestGroup):
    @testcase(bucket="CopterTests2b")
    def GyroFFTMotorNoiseCheck(self):
        """Use FFT to detect post-filter motor noise."""
        # basic gyro sample rate test
        self.progress("Flying with FFT motor-noise detection - Gyro sample rate")
        # This set of parameters creates two noise peaks one at the motor frequency and one at 250Hz
        # we then use ESC telemetry to drive the notch to clean up the motor noise and a post-filter
        # FFT notch to clean up the remaining 250Hz. If either notch fails then the test will be failed
        # due to too much noise being present
        self.set_parameters({
            "AHRS_EKF_TYPE": 10,    # magic tridge EKF type that dramatically speeds up the test
            "EK2_ENABLE": 0,
            "EK3_ENABLE": 0,
            "INS_LOG_BAT_MASK": 3,
            "INS_LOG_BAT_OPT": 4,
            "INS_GYRO_FILTER": 100,
            "INS_FAST_SAMPLE": 3,
            "LOG_BITMASK": 958,
            "LOG_DISARMED": 0,
            "SIM_DRIFT_SPEED": 0,
            "SIM_DRIFT_TIME": 0,
            "SIM_GYR1_RND": 200,     # enable a noisy gyro
            "INS_HNTCH_ENABLE": 1,
            "INS_HNTCH_FREQ": 80,
            "INS_HNTCH_REF": 1.0,
            "INS_HNTCH_HMNCS": 1,   # first harmonic
            "INS_HNTCH_ATT": 50,
            "INS_HNTCH_BW": 30,
            "INS_HNTCH_MODE": 3,    # ESC telemetry
            "INS_HNTCH_OPTS": 2,    # notch-per-motor
            "INS_HNTC2_ENABLE": 1,
            "INS_HNTC2_FREQ": 80,
            "INS_HNTC2_REF": 1.0,
            "INS_HNTC2_HMNCS": 1,
            "INS_HNTC2_ATT": 50,
            "INS_HNTC2_BW": 40,
            "INS_HNTC2_MODE": 0,    # istatic notch
            "INS_HNTC2_OPTS": 16,   # triple-notch
            "FFT_ENABLE": 1,
            "FFT_WINDOW_SIZE": 64,  # not the default, but makes the test more reliable
            "FFT_OPTIONS": 3,
            "FFT_MINHZ": 50,
            "FFT_MAXHZ": 450,
            "SIM_VIB_MOT_MAX": 250, # gives a motor peak at about 145Hz
            "SIM_VIB_FREQ_X": 250,  # create another peak at 250hz
            "SIM_VIB_FREQ_Y": 250,
            "SIM_VIB_FREQ_Z": 250,
            "SIM_GYR_FILE_RW": 2,   # write data to a file
        })
        self.reboot_sitl()

        # the FFT reports its finding during the hover below - before
        # this wait begins - so collect from here rather than only
        # seeing what arrives once we start looking
        self.context_collect('STATUSTEXT')

        # do test flight:
        self.takeoff(10, mode="ALT_HOLD")
        tstart, tend, hover_throttle = self.hover_for_interval(10)
        self.wait_statustext("Noise ", timeout=20, check_context=True)
        self.set_parameter("SIM_GYR1_RND", 0) # stop noise so that we can get home
        self.do_RTL()

        # prevent update parameters from messing with the settings when we pop the context
        self.set_parameters({
            "SIM_VIB_FREQ_X": 0,
            "SIM_VIB_FREQ_Y": 0,
            "SIM_VIB_FREQ_Z": 0,
            "SIM_VIB_MOT_MULT": 1.0,
            "SIM_GYR_FILE_RW": 0,  # stop writing data
            "FFT_ENABLE": 0,
        })
