'''
Helicopter autotest: HeliQuadInvertedFlight

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class HeliQuadInvertedFlight(TestGroup):
    @testcase(bucket="Helicopter")
    def HeliQuadInvertedFlight(self):
        '''fly inverted on collective-pitch quad frame'''
        self.customise_SITL_commandline(
            [],
            defaults_filepath=self.model_defaults_filepath('heli-quad'),
            model="heli-quad:@ROMFS/models/heliquad.json",
            wipe=True,
        )
        # the four collective servos must swing to negative blade pitch
        self.fly_inverted_flight(collective_servos=(1, 2, 3, 4))
        self.do_RTL()
