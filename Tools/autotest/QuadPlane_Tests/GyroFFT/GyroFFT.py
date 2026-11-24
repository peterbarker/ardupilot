'''
QuadPlane autotest: GyroFFT

AP_FLAKE8_CLEAN
'''

import numpy

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class GyroFFT(TestGroup):
    @testcase(bucket="QuadPlaneTests1c")
    def GyroFFT(self):
        """Use dynamic harmonic notch to control motor noise."""
        # basic gyro sample rate test
        self.progress("Flying with gyro FFT - Gyro sample rate")

        # magic tridge EKF type that dramatically speeds up the test
        self.set_parameters({
            "AHRS_EKF_TYPE": 10,

            "INS_LOG_BAT_MASK": 3,
            "INS_LOG_BAT_OPT": 0,
            "INS_GYRO_FILTER": 100,
            "LOG_BITMASK": 45054,
            "LOG_DISARMED": 0,
            "SIM_DRIFT_SPEED": 0,
            "SIM_DRIFT_TIME": 0,
            # enable a noisy motor peak
            "SIM_GYR1_RND": 20,
            # enabling FFT will also enable the arming check: self-testing the functionality
            "FFT_ENABLE": 1,
            "FFT_MINHZ": 80,
            "FFT_MAXHZ": 350,
            "FFT_SNR_REF": 10,
            "FFT_WINDOW_SIZE": 128,
            "FFT_WINDOW_OLAP": 0.75,
        })
        # Step 1: inject a very precise noise peak at 250hz and make sure the in-flight fft
        # can detect it really accurately. For a 128 FFT the frequency resolution is 8Hz so
        # a 250Hz peak should be detectable within 5%
        self.set_parameters({
            "SIM_VIB_FREQ_X": 250,
            "SIM_VIB_FREQ_Y": 250,
            "SIM_VIB_FREQ_Z": 250,
        })
        self.reboot_sitl()

        # find a motor peak
        self.hover_and_check_matched_frequency(-15, 100, 350, 128, 250)

        # Step 2: inject actual motor noise and use the standard length FFT to track it
        self.set_parameters({
            "SIM_VIB_MOT_MAX": 350,
            "FFT_WINDOW_SIZE": 32,
            "FFT_WINDOW_OLAP": 0.5,
        })
        self.reboot_sitl()
        # find a motor peak
        freq = self.hover_and_check_matched_frequency(-15, 200, 300, 32)

        # Step 3: add a FFT dynamic notch and check that the peak is squashed
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

        self.takeoff(10, mode="QHOVER")
        # let the vehicle settle into a stable hover before measuring, so the
        # climb-out transient does not leak into the noise-suppression FFT
        self.wait_altitude(8, 12, relative=True, minimum_duration=10)
        hover_time = 15
        ignore_bins = 20

        self.progress("Hovering for %u seconds" % hover_time)
        tstart = self.get_sim_time()
        while self.get_sim_time_cached() < tstart + hover_time:
            self.assert_receive_message('ATTITUDE')
        tend = self.get_sim_time()

        self.do_RTL()
        psd = self.mavfft_fttd(1, 0, tstart * 1.0e6, tend * 1.0e6)
        freq = psd["F"][numpy.argmax(psd["X"][ignore_bins:]) + ignore_bins]
        peakdB = numpy.amax(psd["X"][ignore_bins:])
        if peakdB < -10:
            self.progress("No motor peak, %f at %f dB" % (freq, peakdB))
        else:
            raise NotAchievedException("Detected peak at %f Hz of %.2f dB" % (freq, peakdB))

        # Step 4: take off as a copter land as a plane, make sure we track
        self.progress("Flying with gyro FFT - vtol to plane")
        self.load_mission("quadplane-gyro-mission.txt")
        if self.mavproxy is not None:
            self.mavproxy.send('wp list\n')
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.wait_waypoint(1, 7, max_dist_to_final_wp_m=60, timeout=1200)
        self.wait_disarmed(timeout=120) # give quadplane a long time to land

        # prevent update parameters from messing with the settings when we pop the context
        self.set_parameter("FFT_ENABLE", 0)
        self.reboot_sitl()
