'''
Copter autotest: GuidedWeatherVane

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class GuidedWeatherVane(TestGroup):
    @testcase(bucket="CopterTests2d")
    def GuidedWeatherVane(self):
        '''check Copter Guided mode weathervane option'''
        self.set_parameters({
            'SIM_WIND_SPD': 10,
            'SIM_WIND_DIR': 90,
            'WVANE_ENABLE': 1,
            # the roll deadzone leaves the heading settled short of the wind
            'WVANE_ANG_MIN': 0,
        })
        self.takeoff(20, mode='GUIDED')
        # turn the shortest way; forcing one can need more than the time allowed
        self.guided_achieve_heading(0, accuracy=1)

        self.set_parameter("GUID_OPTIONS", 128)
        self.wait_heading(90, timeout=60, minimum_duration=10)
        self.do_RTL()
