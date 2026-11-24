'''
Copter autotest: FenceAltFrameComparison

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import AltFrame

from test_discovery import TestGroup
from test_discovery import testcase


class FenceAltFrameComparison(TestGroup):
    @testcase(bucket="CopterTests2d")
    def FenceAltFrameComparison(self):
        '''check the alt fence limits are compared in a common frame'''
        # FENCE_ALT_MIN and FENCE_ALT_MAX each carry their own frame, so
        # their raw values can be measured from different datums.  A
        # minimum 20m above home and a maximum of 100m above home leave
        # 80m of usable airspace whichever way the minimum is expressed.
        self.poll_home_position(quiet=False)
        home_alt = self.get_location().get_alt_m(AltFrame.ABSOLUTE)

        self.context_push()
        self.set_parameters({
            "FENCE_ENABLE": 1,
            "FENCE_TYPE": 9,          # min and max altitude
            "FENCE_ALT_MAX": 100,
            "FENCE_ALT_MAX_TP": 1,    # above home
            "FENCE_ALT_MIN": 20,
            "FENCE_ALT_MIN_TP": 1,    # above home
        })
        self.wait_ready_to_arm()

        self.start_subtest("same limits, minimum expressed in absolute frame")
        self.set_parameters({
            "FENCE_ALT_MIN": home_alt + 20,
            "FENCE_ALT_MIN_TP": 0,    # absolute
        })
        # the numbers now read 604 against 100, but they describe the
        # same 20m-above-home floor as above:
        self.wait_ready_to_arm()

        self.start_subtest("absolute minimum which really is above the maximum")
        self.set_parameters({
            "FENCE_ALT_MIN": home_alt + 150,
        })
        self.assert_prearm_failure("FENCE_ALT_MAX < FENCE_ALT_MIN")

        self.start_subtest("back to a sane minimum")
        self.set_parameters({
            "FENCE_ALT_MIN": home_alt + 20,
        })
        self.wait_ready_to_arm()

        self.context_pop()
