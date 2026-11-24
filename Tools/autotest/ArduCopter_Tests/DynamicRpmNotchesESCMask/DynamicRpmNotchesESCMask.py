'''
Copter autotest: DynamicRpmNotchesESCMask

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class DynamicRpmNotchesESCMask(TestGroup):
    def assert_notch_source_count(self, expected, instance=0):
        '''check the number of frequency sources a harmonic notch instance is
        tracking, as logged in the NDn field of FTN.  Note that FTN is only
        logged when the notch has more than one frequency source, so
        notch-per-motor must be enabled'''
        mlog = self.dfreader_for_current_onboard_log()
        count = None
        while True:
            m = mlog.recv_match(type="FTN")
            if m is None:
                break
            if m.I != instance:
                continue
            count = m.NDn
        if count is None:
            raise NotAchievedException("Did not find a FTN message for notch %u" % instance)
        if count != expected:
            raise NotAchievedException(
                "Expected %u notch sources for notch %u, got %u" %
                (expected, instance, count))
        self.progress("Notch %u is tracking %u sources" % (instance, count))

    @testcase(bucket="CopterTests1a")
    def DynamicRpmNotchesESCMask(self):
        """Check INS_HNTCH_ESCMSK excludes motors from the ESC telemetry notch."""
        self.progress("Checking ESC mask for ESC telemetry driven dynamic notches")

        self.set_rc_default()
        self.set_parameters({
            "LOG_BITMASK": 958,
            "LOG_DISARMED": 0,
            "SIM_ESC_TELEM": 1,
            "INS_HNTCH_ENABLE": 1,
            "INS_HNTCH_REF": 1.0,
            "INS_HNTCH_MODE": 3, # ESC telemetry
            "INS_HNTCH_OPTS": 2, # notch-per-motor, so FTN and FCN are logged
        })

        # the default frame is a quad on outputs 1 to 4.  A mask of zero means
        # use every ESC, masking down to the first two should halve the notches
        for (esc_mask, sources) in [(0, 4), (0b0011, 2)]:
            self.progress("Expecting %u notches with INS_HNTCH_ESCMSK=%u" % (sources, esc_mask))
            self.set_parameter("INS_HNTCH_ESCMSK", esc_mask)
            self.reboot_sitl()

            self.takeoff(10, mode="ALT_HOLD")
            self.hover_for_interval(5)
            self.do_RTL()

            # note that the allocated filter count in FCN cannot be checked
            # here: before ESC telemetry arrives the notch falls back to
            # throttle tracking, which is a notch per motor regardless of the
            # mask, and the filter array never shrinks again
            self.assert_notch_source_count(sources, 0)
