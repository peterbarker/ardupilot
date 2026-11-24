'''
Copter autotest: GyroFFTAverage

AP_FLAKE8_CLEAN
'''

import math

import numpy

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class GyroFFTAverage(TestGroup):
    @testcase(bucket="CopterTests2c", speedup=8)
    def GyroFFTAverage(self):
        """Use dynamic harmonic notch to control motor noise setup via FFT averaging."""
        # the point of this test is that the vehicle works the notch out
        # and saves it, which the suite has no record of and so cannot
        # revert; register the values now so that it can
        self.context_preserve_parameters(self.harmonic_notch_params)
        # basic gyro sample rate test
        self.progress("Flying with gyro FFT harmonic - Gyro sample rate")
        # Step 1
        self.start_subtest("Hover to calculate approximate hover frequency and see that it is tracked")
        # magic tridge EKF type that dramatically speeds up the test
        self.set_parameters({
            "INS_HNTCH_ATT": 100,
            "AHRS_EKF_TYPE": 10,
            "EK2_ENABLE": 0,
            "EK3_ENABLE": 0,
            "INS_LOG_BAT_MASK": 3,
            "INS_LOG_BAT_OPT": 2,
            "INS_GYRO_FILTER": 100,
            "INS_FAST_SAMPLE": 0,
            "LOG_BITMASK": 958,
            "LOG_DISARMED": 0,
            "SIM_DRIFT_SPEED": 0,
            "SIM_DRIFT_TIME": 0,
            "SIM_GYR1_RND": 20,  # enable a noisy gyro
        })
        # motor peak enabling FFT will also enable the arming
        # check, self-testing the functionality
        self.set_parameters({
            "FFT_ENABLE": 1,
            "FFT_WINDOW_SIZE": 64,  # not the default, but makes the test more reliable
            "FFT_SNR_REF": 10,
            "FFT_MINHZ": 80,
            "FFT_MAXHZ": 450,
        })

        # Step 1: inject actual motor noise and use the FFT to track it
        self.set_parameters({
            "SIM_VIB_MOT_MAX": 250, # gives a motor peak at about 175Hz
            "RC7_OPTION" : 162,   # FFT tune
        })

        self.reboot_sitl()

        # hover and engage FFT tracker
        self.takeoff(10, mode="ALT_HOLD")

        hover_time = 60

        # start the tune
        self.set_rc(7, 2000)

        tstart, tend, hover_throttle = self.hover_for_interval(hover_time)

        # finish the tune
        self.set_rc(7, 1000)

        psd = self.mavfft_fttd(1, 0, tstart * 1.0e6, tend * 1.0e6)

        # batch sampler defaults give 1024 fft and sample rate of 1kz so roughly 1hz/bin
        freq = psd["F"][numpy.argmax(psd["X"][50:450]) + 50] * (1000. / 1024.)

        detected_ref = self.get_parameter("INS_HNTCH_REF")
        detected_freq = self.get_parameter("INS_HNTCH_FREQ")
        self.progress("FFT detected parameters were %fHz, ref %f" % (detected_freq, detected_ref))

        # approximate the scaled frequency
        scaled_freq_at_hover = math.sqrt((hover_throttle / 100.) / detected_ref) * detected_freq

        # Check we matched
        if abs(scaled_freq_at_hover - freq) / scaled_freq_at_hover > 0.05:
            raise NotAchievedException("Detected frequency %fHz did not match required %fHz" %
                                       (scaled_freq_at_hover, freq))

        if self.get_parameter("INS_HNTCH_ENABLE") != 1:
            raise NotAchievedException("Harmonic notch was not enabled")

        # Step 2: now rerun the test and check that the peak is squashed
        self.start_subtest("Verify that noise is suppressed by the harmonic notch")
        self.hover_and_check_matched_frequency_with_fft(0, 100, 350, reverse=True, takeoff=False)

        # reset notch to defaults
        self.set_parameters({
            "INS_HNTCH_HMNCS": 3.0,
            "INS_HNTCH_ENABLE": 0.0,
            "INS_HNTCH_REF": 0.0,
            "INS_HNTCH_FREQ": 80,
            "INS_HNTCH_BW": 40,
            "INS_HNTCH_FM_RAT": 1.0
        })

        # Step 3: add a second harmonic and check the first is still tracked
        self.start_subtest("Add a fixed frequency harmonic at twice the hover frequency "
                           "and check the right harmonic is found")
        self.set_parameters({
            "SIM_VIB_FREQ_X": detected_freq * 2,
            "SIM_VIB_FREQ_Y": detected_freq * 2,
            "SIM_VIB_FREQ_Z": detected_freq * 2,
            "SIM_VIB_MOT_MULT": 0.25,  # halve the motor noise so that the higher harmonic dominates
        })
        self.reboot_sitl()

        # hover and engage FFT tracker
        self.takeoff(10, mode="ALT_HOLD")

        hover_time = 60

        # start the tune
        self.set_rc(7, 2000)

        tstart, tend, hover_throttle = self.hover_for_interval(hover_time)

        # finish the tune
        self.set_rc(7, 1000)

        self.do_RTL()

        detected_ref = self.get_parameter("INS_HNTCH_REF")
        detected_freq = self.get_parameter("INS_HNTCH_FREQ")
        self.progress("FFT detected parameters were %fHz, ref %f" % (detected_freq, detected_ref))

        # approximate the scaled frequency
        scaled_freq_at_hover = math.sqrt((hover_throttle / 100.) / detected_ref) * detected_freq

        # Check we matched
        if abs(scaled_freq_at_hover - freq) / scaled_freq_at_hover > 0.05:
            raise NotAchievedException("Detected frequency %fHz did not match required %fHz" %
                                       (scaled_freq_at_hover, freq))

        if self.get_parameter("INS_HNTCH_ENABLE") != 1:
            raise NotAchievedException("Harmonic notch was not enabled")

        self.set_parameters({
            "SIM_VIB_FREQ_X": 0,
            "SIM_VIB_FREQ_Y": 0,
            "SIM_VIB_FREQ_Z": 0,
            "SIM_VIB_MOT_MULT": 1.0,
            "INS_HNTCH_HMNCS": 3.0,
            "INS_HNTCH_ENABLE": 0.0,
            "INS_HNTCH_REF": 0.0,
            "INS_HNTCH_FREQ": 80,
            "INS_HNTCH_BW": 40,
            "INS_HNTCH_FM_RAT": 1.0
        })
        # prevent update parameters from messing with the settings when we pop the context
        self.set_parameter("FFT_ENABLE", 0)
