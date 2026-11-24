'''
Copter autotest: PILOT_THR_BHV

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import WaitAndMaintainArmed

from test_discovery import TestGroup
from test_discovery import testcase


class PILOT_THR_BHV(TestGroup):
    @testcase(bucket="CopterTests2d")
    def PILOT_THR_BHV(self):
        '''test the PILOT_THR_BHV parameter'''
        self.start_subtest("Test default behaviour, no disarm on land")
        self.set_parameters({
            "DISARM_DELAY": 0,
        })
        self.takeoff(2, mode='GUIDED')
        self.hover()
        self.change_mode('LOITER')
        self.set_rc(3, 1300)

        maintain_armed = WaitAndMaintainArmed(self, minimum_duration=20)
        maintain_armed.run()

        self.start_subtest("Test THR_BEHAVE_DISARM_ON_LAND_DETECT")
        self.set_parameters({
            "PILOT_THR_BHV": 4,  # Disarm on land detection
        })
        self.zero_throttle()
        self.takeoff(2, mode='GUIDED')
        self.hover()
        self.change_mode('LOITER')
        self.set_rc(3, 1300)

        self.wait_disarmed()
