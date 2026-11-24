'''
Plane autotest: AUTOTUNE

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class AUTOTUNE(TestGroup):
    @testcase(bucket="PlaneTests1c")
    def AUTOTUNE(self):
        '''Test AutoTune mode'''
        self.context_preserve_parameters(self.autotune_saved_gains)

        # Prior to 2026-07-15, the SITL airspeed sensor was reading about 5%
        # slower than actual during this test. We bump the scaling airspeed by
        # that same amount to get the same tune results as before.
        self.set_parameter("SCALING_SPEED", 15.8)

        self.run_autotune()

        # Values that are set to constants
        # If these are changed then the expected tune parameters should also change
        self.assert_parameter_value_pct("RLL2SRV_TCONST", 0.5, 0)
        self.assert_parameter_value_pct("RLL2SRV_RMAX", 75, 0)
        self.assert_parameter_value_pct("RLL_RATE_IMAX", 0.666, 0.01) # allow some small error to account for floating point stuff  # noqa:E501
        self.assert_parameter_value_pct("RLL_RATE_FLTT", 3.183, 0.01)
        self.assert_parameter_value_pct("RLL_RATE_FLTE", 0, 0)
        self.assert_parameter_value_pct("RLL_RATE_FLTD", 10.0, 0)
        self.assert_parameter_value_pct("RLL_RATE_SMAX", 150.0, 0)

        self.assert_parameter_value_pct("PTCH2SRV_TCONST", 0.75, 0)
        self.assert_parameter_value_pct("PTCH2SRV_RMAX_UP", 75, 0)
        self.assert_parameter_value_pct("PTCH2SRV_RMAX_DN", 75, 0)
        self.assert_parameter_value_pct("PTCH_RATE_IMAX", 0.666, 0.01)
        self.assert_parameter_value_pct("PTCH_RATE_FLTT", 2.122, 0.01)
        self.assert_parameter_value_pct("PTCH_RATE_FLTE", 0, 0)
        self.assert_parameter_value_pct("PTCH_RATE_FLTD", 10, 0)
        self.assert_parameter_value_pct("PTCH_RATE_SMAX", 150, 0)

        # Check tuned values, targets derived from running tests multiple times and taking average
        # Expect within 2%
        # Note that I is not checked directly, its value is derived from P, FF, and TCONST which are all checked.
        self.assert_parameter_value_pct("RLL_RATE_P", 1.222702146, 2)
        self.assert_parameter_value_pct("RLL_RATE_FF", 0.229291457, 2)
        self.assert_parameter_value_pct("RLL_RATE_D", 0.070284024, 2)

        self.assert_parameter_value_pct("PTCH_RATE_FF", 0.503520715, 5)

        # PTCH_RATE_P solutions, most likely to least likely: 1.746 (~99%), 1.343 (~1%), 2.270 (<1%)
        acceptable_ptch_rate_p = [1.746079683, 1.343138218, 2.26990366]
        ptch_rate_p = self.get_parameter("PTCH_RATE_P")
        if not any(abs(ptch_rate_p - v) <= v * 0.02 for v in acceptable_ptch_rate_p):
            raise NotAchievedException("PTCH_RATE_P=%f is not an expected value" % ptch_rate_p)

        # PTCH_RATE_D solutions, most likely to least likely: 0.108 (~99%), 0.141 (<1%), 0.049 (<1%), 0.0836 (<1%), 0.038 (<1%)
        acceptable_ptch_rate_d = [0.108, 0.141, 0.049, 0.0836, 0.0380]
        ptch_rate_d = self.get_parameter("PTCH_RATE_D")
        if not any(abs(ptch_rate_d - v) <= v * 0.02 for v in acceptable_ptch_rate_d):
            raise NotAchievedException("PTCH_RATE_D=%f is not an expected value" % ptch_rate_d)
