'''
Copter autotest: GyroFFTPostFilter

AP_FLAKE8_CLEAN
'''

import numpy

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class GyroFFTPostFilter(TestGroup):
    @testcase(bucket="CopterTests1d")
    def GyroFFTPostFilter(self):
        """Use FFT-driven dynamic harmonic notch to control post-RPM filter motor noise."""
        # basic gyro sample rate test
        self.progress("Flying with gyro FFT post-filter suppression - Gyro sample rate")
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
            "SIM_GYR1_RND": 20,     # enable a noisy gyro
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
            "INS_HNTC2_MODE": 4,    # in-flight FFT
            "INS_HNTC2_OPTS": 18,   # triple-notch, notch-per-FFT peak
            "FFT_ENABLE": 1,
            "FFT_WINDOW_SIZE": 64,  # not the default, but makes the test more reliable
            "FFT_OPTIONS": 1,
            "FFT_MINHZ": 50,
            "FFT_MAXHZ": 450,
            "SIM_VIB_MOT_MAX": 250, # gives a motor peak at about 145Hz
            "SIM_VIB_FREQ_X": 250,  # create another peak at 250hz
            "SIM_VIB_FREQ_Y": 250,
            "SIM_VIB_FREQ_Z": 250,
            "SIM_GYR_FILE_RW": 2,   # write data to a file
        })
        self.reboot_sitl()

        # do test flight:
        self.takeoff(10, mode="ALT_HOLD")
        tstart, tend, hover_throttle = self.hover_for_interval(60)
        # fly fast forrest!
        self.set_rc_from_map({3: 1900, 2: 1200})
        self.wait_groundspeed(5, 1000)
        self.set_rc_from_map({3: 1500, 2: 1500})
        self.do_RTL()

        psd = self.mavfft_fttd(1, 2, tstart * 1.0e6, tend * 1.0e6)

        # batch sampler defaults give 1024 fft and sample rate of 1kz so roughly 1hz/bin
        scale = 1000. / 1024.
        sminhz = int(100 * scale)
        smaxhz = int(350 * scale)
        freq = psd["F"][numpy.argmax(psd["X"][sminhz:smaxhz]) + sminhz]
        peakdb = numpy.amax(psd["X"][sminhz:smaxhz])
        if peakdb < -5:
            self.progress("Did not detect a motor peak, found %fHz at %fdB" % (freq, peakdb))
        else:
            raise NotAchievedException("Detected %fHz motor peak at %fdB" % (freq, peakdb))

        # prevent update parameters from messing with the settings when we pop the context
        self.set_parameters({
            "SIM_VIB_FREQ_X": 0,
            "SIM_VIB_FREQ_Y": 0,
            "SIM_VIB_FREQ_Z": 0,
            "SIM_VIB_MOT_MULT": 1.0,
            "SIM_GYR_FILE_RW": 0,  # stop writing data
            "FFT_ENABLE": 0,
        })
