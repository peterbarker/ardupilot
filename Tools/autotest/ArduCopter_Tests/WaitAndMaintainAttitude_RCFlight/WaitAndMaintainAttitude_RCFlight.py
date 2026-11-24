'''
Copter autotest: WaitAndMaintainAttitude_RCFlight

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import WaitAndMaintainAttitude

from test_discovery import TestGroup
from test_discovery import testcase


class WaitAndMaintainAttitude_RCFlight(TestGroup):
    @testcase(bucket="CopterTests1c")
    def WaitAndMaintainAttitude_RCFlight(self):
        '''just test WaitAndMaintainAttitude works'''
        WaitAndMaintainAttitude(self, 0, 0, epsilon=1).run()
        self.takeoff(20, mode='LOITER')

        WaitAndMaintainAttitude(self, 0, 0, epsilon=1).run()

        self.start_subtest("Full forward stick")
        self.set_rc(2, 1000)
        WaitAndMaintainAttitude(self, 0, -30, epsilon=1, minimum_duration=10, timeout=60).run()

        self.start_subtest("Full left stick")
        self.set_rc_from_map({2: 1500, 1: 1000})
        WaitAndMaintainAttitude(self, -30, 0, epsilon=1).run()

        self.start_subtest("Circular angle")
        self.set_rc_from_map({2: 1000, 1: 1000})
        # this is some sort of circular-angle shenanigans / frame
        # translation weirdness:
        WaitAndMaintainAttitude(self, -20, -23, epsilon=1, timeout=120).run()

        self.start_subtest("Center the sticks")
        self.set_rc_from_map({2: 1500, 1: 1500})
        WaitAndMaintainAttitude(self, 0, 0, epsilon=1).run()

        self.do_RTL()
