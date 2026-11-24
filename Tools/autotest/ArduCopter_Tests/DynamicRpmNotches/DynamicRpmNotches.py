'''
Copter autotest: DynamicRpmNotches

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class DynamicRpmNotches(TestGroup):
    def get_average_esc_frequency(self):
        mlog = self.dfreader_for_current_onboard_log()
        rpm_total = 0
        rpm_count = 0
        tho = 0
        while True:
            m = mlog.recv_match()
            if m is None:
                break
            msg_type = m.get_type()
            if msg_type == "CTUN":
                tho = m.ThO
            elif msg_type == "ESC" and tho > 0.1:
                rpm_total += m.RPM
                rpm_count += 1

        esc_hz = rpm_total / (rpm_count * 60)
        return esc_hz

    @testcase(bucket="CopterTests1a")  # Do not add attempts to this - failure is sign of a bug
    def DynamicRpmNotches(self):
        """Use dynamic harmonic notch to control motor noise via ESC telemetry."""
        self.progress("Flying with ESC telemetry driven dynamic notches")

        self.set_rc_default()
        self.set_parameters({
            "AHRS_EKF_TYPE": 10,
            "INS_LOG_BAT_MASK": 3,
            "INS_LOG_BAT_OPT": 0,
            "INS_GYRO_FILTER": 300, # set gyro filter high so we can observe behaviour
            "LOG_BITMASK": 958,
            "LOG_DISARMED": 0,
            "SIM_VIB_MOT_MAX": 350,
            "SIM_GYR1_RND": 20,
            "SIM_ESC_TELEM": 1
        })
        self.reboot_sitl()

        self.takeoff(10, mode="ALT_HOLD")

        # find a motor peak, the peak is at about 190Hz, so checking between 50 and 320Hz should be safe.
        # there is a second harmonic at 380Hz which should be avoided to make the test reliable
        # detect at -5dB so we don't pick some random noise as the peak. The actual peak is about +15dB
        freq, hover_throttle, peakdb = self.hover_and_check_matched_frequency_with_fft(-5, 50, 320)

        # now add a dynamic notch and check that the peak is squashed
        self.set_parameters({
            "INS_LOG_BAT_OPT": 4,
            "INS_HNTCH_ENABLE": 1,
            "INS_HNTCH_FREQ": 80,
            "INS_HNTCH_REF": 1.0,
            "INS_HNTCH_HMNCS": 5, # first and third harmonic
            "INS_HNTCH_ATT": 50,
            "INS_HNTCH_BW": 40,
            "INS_HNTCH_MODE": 3,
        })
        self.reboot_sitl()

        # -10dB is pretty conservative - actual is about -25dB
        freq, hover_throttle, peakdb1, psd = \
            self.hover_and_check_matched_frequency_with_fft_and_psd(-10, 50, 320, reverse=True, instance=2)
        # find the noise at the motor frequency
        esc_hz = self.get_average_esc_frequency()
        esc_peakdb1 = psd["X"][int(esc_hz)]

        # now add notch-per motor and check that the peak is squashed
        self.set_parameter("INS_HNTCH_OPTS", 2)
        self.reboot_sitl()

        freq, hover_throttle, peakdb2, psd = \
            self.hover_and_check_matched_frequency_with_fft_and_psd(-10, 50, 320, reverse=True, instance=2)
        # find the noise at the motor frequency
        esc_hz = self.get_average_esc_frequency()
        esc_peakdb2 = psd["X"][int(esc_hz)]

        # notch-per-motor will be better at the average ESC frequency
        if esc_peakdb2 > esc_peakdb1:
            raise NotAchievedException(
                "Notch-per-motor peak was higher than single-notch peak %fdB > %fdB" %
                (esc_peakdb2, esc_peakdb1))

        # check that the noise is being squashed at all. this needs to be an aggressive check so that failure happens easily
        # testing shows this to be -58dB on average
        if esc_peakdb2 > -25:
            raise NotAchievedException(
                "Notch-per-motor had a peak of %fdB there should be none" % esc_peakdb2)

        # Now do it again for an octacopter
        self.progress("Flying Octacopter with ESC telemetry driven dynamic notches")
        self.set_parameter("INS_HNTCH_OPTS", 0)
        self.customise_SITL_commandline(
            [],
            model="octa"
        )
        # search from 150Hz rather than 50Hz: the octa motor fundamental the
        # notch targets is ~190-200Hz, but the high INS_GYRO_FILTER used here
        # lets the rate loop sustain a closed-loop oscillation near half the
        # motor frequency (~90Hz).  That subharmonic is real vehicle motion the
        # harmonic notch is not meant to remove, so don't let it fail the notch
        # check.  Hover longer too, to settle the Welch FFT estimate.
        freq, hover_throttle, peakdb1, psd = \
            self.hover_and_check_matched_frequency_with_fft_and_psd(-10, 150, 320, reverse=True, instance=2, hover_time=30)
        # find the noise at the motor frequency
        esc_hz = self.get_average_esc_frequency()
        esc_peakdb1 = psd["X"][int(esc_hz)]

        # now add notch-per motor and check that the peak is squashed
        self.set_parameter("INS_HNTCH_HMNCS", 1)
        self.set_parameter("INS_HNTCH_OPTS", 2)
        self.reboot_sitl()

        freq, hover_throttle, peakdb2, psd = \
            self.hover_and_check_matched_frequency_with_fft_and_psd(-15, 150, 320, reverse=True, instance=2, hover_time=30)
        # find the noise at the motor frequency
        esc_hz = self.get_average_esc_frequency()
        esc_peakdb2 = psd["X"][int(esc_hz)]

        # notch-per-motor will be better at the average ESC frequency
        if esc_peakdb2 > esc_peakdb1:
            raise NotAchievedException(
                "Notch-per-motor peak was higher than single-notch peak %fdB > %fdB" %
                (esc_peakdb2, esc_peakdb1))

    @testcase(bucket="CopterTests1a")
    def DynamicRpmNotchesRateThread(self):
        """Use dynamic harmonic notch to control motor noise via ESC telemetry."""
        self.progress("Flying with ESC telemetry driven dynamic notches")
        self.context_push()
        self.set_rc_default()
        self.set_parameters({
            "AHRS_EKF_TYPE": 10,
            "INS_LOG_BAT_MASK": 3,
            "INS_LOG_BAT_OPT": 0,
            "INS_GYRO_FILTER": 300, # set gyro filter high so we can observe behaviour
            "LOG_BITMASK": 959,
            "LOG_DISARMED": 0,
            "SIM_VIB_MOT_MAX": 350,
            "SIM_GYR1_RND": 20,
            "SIM_ESC_TELEM": 1,
            "FSTRATE_ENABLE": 1
        })
        self.reboot_sitl()

        self.takeoff(10, mode="ALT_HOLD")

        # find a motor peak, the peak is at about 190Hz, so checking between 50 and 320Hz should be safe.
        # there is a second harmonic at 380Hz which should be avoided to make the test reliable
        # detect at -5dB so we don't pick some random noise as the peak. The actual peak is about +15dB
        freq, hover_throttle, peakdb = self.hover_and_check_matched_frequency_with_fft(-5, 50, 320)

        # now add a dynamic notch and check that the peak is squashed
        self.set_parameters({
            "INS_LOG_BAT_OPT": 4,
            "INS_HNTCH_ENABLE": 1,
            "INS_HNTCH_FREQ": 80,
            "INS_HNTCH_REF": 1.0,
            "INS_HNTCH_HMNCS": 5, # first and third harmonic
            "INS_HNTCH_ATT": 50,
            "INS_HNTCH_BW": 40,
            "INS_HNTCH_MODE": 3,
            "FSTRATE_ENABLE": 1
        })
        self.reboot_sitl()

        # -10dB is pretty conservative - actual is about -25dB
        freq, hover_throttle, peakdb1, psd = \
            self.hover_and_check_matched_frequency_with_fft_and_psd(-10, 50, 320, reverse=True, instance=2)
        # find the noise at the motor frequency
        esc_hz = self.get_average_esc_frequency()
        esc_peakdb1 = psd["X"][int(esc_hz)]

        # now add notch-per motor and check that the peak is squashed
        self.set_parameter("INS_HNTCH_OPTS", 2)
        self.reboot_sitl()

        freq, hover_throttle, peakdb2, psd = \
            self.hover_and_check_matched_frequency_with_fft_and_psd(-10, 50, 320, reverse=True, instance=2)
        # find the noise at the motor frequency
        esc_hz = self.get_average_esc_frequency()
        esc_peakdb2 = psd["X"][int(esc_hz)]

        # notch-per-motor will be better at the average ESC frequency
        if esc_peakdb2 > esc_peakdb1:
            raise NotAchievedException(
                "Notch-per-motor peak was higher than single-notch peak %fdB > %fdB" %
                (esc_peakdb2, esc_peakdb1))

        # check that the noise is being squashed at all. this needs to be an aggressive check so that failure happens easily
        # testing shows this to be -58dB on average
        if esc_peakdb2 > -25:
            raise NotAchievedException(
                "Notch-per-motor had a peak of %fdB there should be none" % esc_peakdb2)
        self.context_pop()
        self.reboot_sitl()
