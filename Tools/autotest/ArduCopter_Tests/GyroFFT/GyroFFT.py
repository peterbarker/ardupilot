'''
Copter autotest: GyroFFT

AP_FLAKE8_CLEAN
'''

import numpy

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class GyroFFT(TestGroup):
    @testcase(bucket="CopterTests2d", speedup=8)
    def GyroFFT(self):
        """Use dynamic harmonic notch to control motor noise."""
        # basic gyro sample rate test
        self.progress("Flying with gyro FFT - Gyro sample rate")

        # magic tridge EKF type that dramatically speeds up the test
        self.set_parameters({
            "AHRS_EKF_TYPE": 10,
            "EK2_ENABLE": 0,
            "EK3_ENABLE": 0,
            "INS_LOG_BAT_MASK": 3,
            "INS_LOG_BAT_OPT": 4,
            "INS_GYRO_FILTER": 100,
            "INS_FAST_SAMPLE": 0,
            "LOG_BITMASK": 958,
            "LOG_DISARMED": 0,
            "SIM_DRIFT_SPEED": 0,
            "SIM_DRIFT_TIME": 0,
            "SIM_GYR1_RND": 20,  # enable a noisy motor peak
        })
        # enabling FFT will also enable the arming check,
        # self-testing the functionality
        self.set_parameters({
            "FFT_ENABLE": 1,
            "FFT_MINHZ": 50,
            "FFT_MAXHZ": 450,
            "FFT_SNR_REF": 10,
            "FFT_WINDOW_SIZE": 128,
            "FFT_WINDOW_OLAP": 0.75,
            "FFT_SAMPLE_MODE": 0,
        })

        # Step 1: inject a very precise noise peak at 250hz and make sure the in-flight fft
        # can detect it really accurately. For a 128 FFT the frequency resolution is 8Hz so
        # a 250Hz peak should be detectable within 5%
        self.start_subtest("Inject noise at 250Hz and check the FFT can find the noise")
        self.set_parameters({
            "SIM_VIB_FREQ_X": 250,
            "SIM_VIB_FREQ_Y": 250,
            "SIM_VIB_FREQ_Z": 250,
        })

        self.reboot_sitl()

        # find a motor peak
        self.hover_and_check_matched_frequency(
            dblevel=-15,
            minhz=100,
            maxhz=350,
            fftLength=128,
            peakhz=250,
        )

        # Step 1b: run the same test with an FFT length of 256 which is needed to flush out a
        # whole host of bugs related to uint8_t. This also tests very accurately the frequency resolution
        self.set_parameter("FFT_WINDOW_SIZE", 256)
        self.start_subtest("Inject noise at 250Hz and check the FFT can find the noise")

        self.reboot_sitl()

        # find a motor peak
        self.hover_and_check_matched_frequency(
            dblevel=-15,
            minhz=100,
            maxhz=350,
            fftLength=256,
            peakhz=250,
        )
        self.set_parameter("FFT_WINDOW_SIZE", 128)

        # Step 2: inject actual motor noise and use the standard length FFT to track it
        self.start_subtest("Hover and check that the FFT can find the motor noise")
        self.set_parameters({
            "SIM_VIB_FREQ_X": 0,
            "SIM_VIB_FREQ_Y": 0,
            "SIM_VIB_FREQ_Z": 0,
            "SIM_VIB_MOT_MAX": 250,  # gives a motor peak at about 175Hz
            "FFT_WINDOW_SIZE": 32,
            "FFT_WINDOW_OLAP": 0.5,
        })

        self.reboot_sitl()
        freq = self.hover_and_check_matched_frequency(
            dblevel=-15,
            minhz=100,
            maxhz=250,
            fftLength=32,
        )

        self.set_parameter("SIM_VIB_MOT_MULT", 1.)

        # Step 3: add a FFT dynamic notch and check that the peak is squashed
        self.start_subtest("Add a dynamic notch, hover and check that the noise peak is now gone")
        self.set_parameters({
            "INS_LOG_BAT_OPT": 2,
            "INS_HNTCH_ENABLE": 1,
            "INS_HNTCH_FREQ": freq,
            "INS_HNTCH_REF": 1.0,
            "INS_HNTCH_ATT": 50,
            "INS_HNTCH_BW": freq/2,
            "INS_HNTCH_MODE": 4,
        })
        self.reboot_sitl()

        # do test flight:
        self.takeoff(10, mode="ALT_HOLD")
        # let the vehicle settle into a stable hover before measuring, so the
        # climb-out transient does not leak into the noise-suppression FFT
        self.wait_altitude(8, 12, relative=True, minimum_duration=10)
        tstart, tend, hover_throttle = self.hover_for_interval(15)
        # fly fast forrest!
        self.set_rc_from_map({3: 1900, 2: 1200})
        self.wait_groundspeed(5, 1000)
        self.set_rc_from_map({3: 1500, 2: 1500})
        self.do_RTL()

        psd = self.mavfft_fttd(1, 0, tstart * 1.0e6, tend * 1.0e6)

        # batch sampler defaults give 1024 fft and sample rate of 1kz so roughly 1hz/bin
        scale = 1000. / 1024.
        sminhz = int(100 * scale)
        smaxhz = int(350 * scale)
        freq = psd["F"][numpy.argmax(psd["X"][sminhz:smaxhz]) + sminhz]
        peakdb = numpy.amax(psd["X"][sminhz:smaxhz])
        if peakdb < 0:
            self.progress("Did not detect a motor peak, found %fHz at %fdB" % (freq, peakdb))
        else:
            raise NotAchievedException("Detected %fHz motor peak at %fdB" % (freq, peakdb))

        # Step 4: loop sample rate test with larger window
        self.start_subtest("Hover and check that the FFT can find the motor noise when running at fast loop rate")
        # we are limited to half the loop rate for frequency detection
        self.set_parameters({
            "FFT_MAXHZ": 185,
            "INS_LOG_BAT_OPT": 4,
            "SIM_VIB_MOT_MAX": 220,
            "FFT_WINDOW_SIZE": 64,
            "FFT_WINDOW_OLAP": 0.75,
            "FFT_SAMPLE_MODE": 1,
        })
        self.reboot_sitl()

        # do test flight:
        self.takeoff(10, mode="ALT_HOLD")
        tstart, tend, hover_throttle = self.hover_for_interval(15)
        self.do_RTL()

        # why are we not checking the results from that flight? -pb20220613

        # prevent update parameters from messing with the settings
        # when we pop the context
        self.set_parameter("FFT_ENABLE", 0)
