'''
Helicopter autotest: HeliQuad

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class HeliQuad(TestGroup):
    @testcase(bucket="Helicopter")
    def HeliQuad(self):
        '''fly collective-pitch quad frame'''
        self.customise_SITL_commandline(
            [],
            defaults_filepath=self.model_defaults_filepath('heli-quad'),
            model="heli-quad:@ROMFS/models/heliquad.json",
            wipe=True,
        )
        self.takeoff(10)
        self.do_RTL()
