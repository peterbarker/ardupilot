'''
Copter autotest: LoiterToGuidedHomeVSOrigin

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import AltFrame

from test_discovery import TestGroup
from test_discovery import testcase


class LoiterToGuidedHomeVSOrigin(TestGroup):
    @testcase(bucket="CopterTests2b")
    def LoiterToGuidedHomeVSOrigin(self):
        '''test moving from guided to loiter mode when home is a different alt
        to origin'''
        self.set_parameters({
            "TERRAIN_ENABLE": 1,
            "SIM_TERRAIN": 1,
        })
        self.takeoff(10, mode='GUIDED')
        here = self.get_location()
        here_alt_amsl = here.get_alt_m(AltFrame.ABSOLUTE)
        self.set_home(here)
        self.hover()
        self.change_mode('LOITER')
        self.wait_altitude(here_alt_amsl-1, here_alt_amsl+1, minimum_duration=10)
        self.disarm_vehicle(force=True)
