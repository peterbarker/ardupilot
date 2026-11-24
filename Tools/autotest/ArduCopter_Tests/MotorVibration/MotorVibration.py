'''
Copter autotest: MotorVibration

AP_FLAKE8_CLEAN
'''

import numpy

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class MotorVibration(TestGroup):
    @testcase(bucket="CopterTests2b")
    def MotorVibration(self):
        """Test flight with motor vibration"""
        # magic tridge EKF type that dramatically speeds up the test
        self.set_parameters({
            "AHRS_EKF_TYPE": 10,
            "INS_LOG_BAT_MASK": 3,
            "INS_LOG_BAT_OPT": 0,
            "LOG_BITMASK": 958,
            "LOG_DISARMED": 0,
            "SIM_VIB_MOT_MAX": 350,
            # these are real values taken from a 180mm Quad:
            "SIM_GYR1_RND": 20,
            "SIM_ACC1_RND": 5,
            "SIM_ACC2_RND": 5,
            "SIM_INS_THR_MIN": 0.1,
        })
        self.reboot_sitl()

        # do a simple up-and-down flight to gather data:
        self.takeoff(15, mode="ALT_HOLD")
        tstart, tend, hover_throttle = self.hover_for_interval(15)
        # if we don't reduce vibes here then the landing detector
        # may not trigger
        self.set_parameter("SIM_VIB_MOT_MAX", 0)
        self.do_RTL()

        psd = self.mavfft_fttd(1, 0, tstart * 1.0e6, tend * 1.0e6)
        # ignore the first 20Hz and look for a peak at -15dB or more
        # it should be at about 190Hz, each bin is 1000/1024Hz wide
        ignore_bins = int(100 * 1.024)  # start at 100Hz to be safe
        freq = psd["F"][numpy.argmax(psd["X"][ignore_bins:]) + ignore_bins]
        if numpy.amax(psd["X"][ignore_bins:]) < -15 or freq < 100 or freq > 300:
            raise NotAchievedException(
                "Did not detect a motor peak, found %f at %f dB" %
                (freq, numpy.amax(psd["X"][ignore_bins:])))
        else:
            self.progress("Detected motor peak at %fHz" % freq)

        # now add a notch and check that post-filter the peak is squashed below 40dB
        self.set_parameters({
            "INS_LOG_BAT_OPT": 2,
            "INS_HNTC2_ENABLE": 1,
            "INS_HNTC2_FREQ": freq,
            "INS_HNTC2_ATT": 50,
            "INS_HNTC2_BW": freq/2,
            "INS_HNTC2_MODE": 0,
            "SIM_VIB_MOT_MAX": 350,
        })
        self.reboot_sitl()

        # do a simple up-and-down flight to gather data:
        self.takeoff(15, mode="ALT_HOLD")
        # let the vehicle settle into a stable hover before measuring, so the
        # climb-out transient does not leak into the noise-suppression FFT
        self.wait_altitude(13, 17, relative=True, minimum_duration=10)
        tstart, tend, hover_throttle = self.hover_for_interval(15)
        self.set_parameter("SIM_VIB_MOT_MAX", 0)
        self.do_RTL()

        psd = self.mavfft_fttd(1, 0, tstart * 1.0e6, tend * 1.0e6)
        freq = psd["F"][numpy.argmax(psd["X"][ignore_bins:]) + ignore_bins]
        peakdB = numpy.amax(psd["X"][ignore_bins:])
        if peakdB < -23:
            self.progress("Did not detect a motor peak, found %f at %f dB" % (freq, peakdB))
        else:
            raise NotAchievedException("Detected peak %.1f Hz %.2f dB" % (freq, peakdB))
