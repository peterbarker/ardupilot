'''
Helicopter autotest: HeliSingleInvertedFlight

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class HeliSingleInvertedFlight(TestGroup):
    @testcase(bucket="Helicopter")
    def HeliSingleInvertedFlight(self):
        '''attempt inverted flight on heli single frame and check the set that
        succeeds matches expectations'''
        # parameters that let the traditional heli frames push the collective
        # negative far enough to hold inverted flight; set for this test only.
        # IM_STB_COL_* only affect STABILIZE collective; the maneuver flies in
        # ALT_HOLD so they are belt-and-braces.
        inverted_params = {
            "H_COL_MIN": 1260,
            "H_COL_ANG_MIN": -12,
            "ATC_RATE_R_MAX": 120,
            "ATC_RATE_P_MAX": 120,
            "IM_STB_COL_1": 40,
            "IM_STB_COL_2": 70,
            "IM_STB_COL_3": 80,
        }
        self.customise_SITL_commandline(
            [],
            defaults_filepath=self.model_defaults_filepath('heli'),
            model="heli",
            wipe=True,
        )
        self.set_parameters(inverted_params)
        self.fly_inverted_flight(yaw_tolerance=30)
        self.do_RTL()
