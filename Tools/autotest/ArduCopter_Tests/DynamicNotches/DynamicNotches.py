'''
Copter autotest: DynamicNotches

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class DynamicNotches(TestGroup):
    def assert_notch_filter_count(self, expected, instance=0):
        '''check the number of notch filters allocated for a harmonic notch
        instance, as logged in the NF field of FCN.  Note that FCN is only
        logged when the notch has more than one frequency source, so
        notch-per-motor must be enabled'''
        mlog = self.dfreader_for_current_onboard_log()
        count = None
        while True:
            m = mlog.recv_match(type="FCN")
            if m is None:
                break
            if m.I != instance:
                continue
            count = m.NF
        if count is None:
            raise NotAchievedException("Did not find a FCN message for notch %u" % instance)
        if count != expected:
            raise NotAchievedException(
                "Expected %u notch filters for notch %u, got %u" %
                (expected, instance, count))
        self.progress("Notch %u has %u filters" % (instance, count))

    @testcase(bucket="CopterTests2b", attempts=4)
    def DynamicNotches(self):
        """Use dynamic harmonic notch to control motor noise."""
        self.progress("Flying with dynamic notches")

        self.set_parameters({
            "AHRS_EKF_TYPE": 10,
            "INS_LOG_BAT_MASK": 3,
            "INS_LOG_BAT_OPT": 0,
            "INS_GYRO_FILTER": 100, # set the gyro filter high so we can observe behaviour
            "LOG_BITMASK": 958,
            "LOG_DISARMED": 0,
            "SIM_VIB_MOT_MAX": 350,
            "SIM_GYR1_RND": 20,
        })
        self.reboot_sitl()

        self.takeoff(10, mode="ALT_HOLD")

        # find a motor peak
        freq, hover_throttle, peakdb = self.hover_and_check_matched_frequency_with_fft(-15, 200, 300)

        # now add a dynamic notch and check that the peak is squashed
        self.set_parameters({
            "INS_LOG_BAT_OPT": 2,
            "INS_HNTCH_ENABLE": 1,
            "INS_HNTCH_FREQ": freq,
            "INS_HNTCH_REF": hover_throttle/100.,
            "INS_HNTCH_HMNCS": 5, # first and third harmonic
            "INS_HNTCH_ATT": 50,
            "INS_HNTCH_BW": freq/2,
        })
        self.reboot_sitl()

        freq, hover_throttle, peakdb1 = \
            self.hover_and_check_matched_frequency_with_fft(-10, 20, 350, reverse=True)

        # a peak check alone cannot tell a quintuple notch from a triple one, so
        # notch-per-motor is enabled for the composite notch runs below.  That
        # makes the number of allocated filters visible in the NF field of FCN,
        # which is only logged for a multi-source notch.  Note this also moves
        # throttle tracking onto per-motor thrust, so the runs below fly a notch
        # per motor rather than a single throttle-derived notch.
        motors = 4      # the default frame is a quad
        harmonics = 2   # INS_HNTCH_HMNCS is 5, the first and third harmonic

        # now add double dynamic notches and check that the peak is squashed
        self.set_parameter("INS_HNTCH_OPTS", 3)  # double-notch, notch-per-motor
        self.reboot_sitl()

        freq, hover_throttle, peakdb2 = \
            self.hover_and_check_matched_frequency_with_fft(-15, 20, 350, reverse=True)

        self.assert_notch_filter_count(motors * harmonics * 2)

        # double-notch should do better, but check for within 5%
        if peakdb2 * 1.05 > peakdb1:
            raise NotAchievedException(
                "Double-notch peak was higher than single-notch peak %fdB > %fdB" %
                (peakdb2, peakdb1))

        # now add triple dynamic notches and check that the peak is squashed
        self.set_parameter("INS_HNTCH_OPTS", 18)  # triple-notch, notch-per-motor
        self.reboot_sitl()

        freq, hover_throttle, peakdb2 = \
            self.hover_and_check_matched_frequency_with_fft(-15, 20, 350, reverse=True)

        self.assert_notch_filter_count(motors * harmonics * 3)

        # triple-notch should do better, but check for within 5%
        if peakdb2 * 1.05 > peakdb1:
            raise NotAchievedException(
                "Triple-notch peak was higher than single-notch peak %fdB > %fdB" %
                (peakdb2, peakdb1))

        # now add quintuple dynamic notches and check that the peak is squashed
        self.set_parameter("INS_HNTCH_OPTS", 66)  # quintuple-notch, notch-per-motor
        self.reboot_sitl()

        freq, hover_throttle, peakdb2 = \
            self.hover_and_check_matched_frequency_with_fft(-15, 20, 350, reverse=True)

        self.assert_notch_filter_count(motors * harmonics * 5)

        # quintuple-notch should do better, but check for within 5%
        if peakdb2 * 1.05 > peakdb1:
            raise NotAchievedException(
                "Quintuple-notch peak was higher than single-notch peak %fdB > %fdB" %
                (peakdb2, peakdb1))
