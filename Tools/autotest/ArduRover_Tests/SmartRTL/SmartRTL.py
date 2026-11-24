'''
Rover autotest: SmartRTL

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class SmartRTL(TestGroup):
    @testcase(bucket="Rover")
    def SmartRTL(self):
        '''Test SmartRTL'''
        self.change_mode("STEERING")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        # drive two sides of a square, make sure we don't go back through
        # the middle of the square
        self.progress("Driving North")
        self.reach_heading_manual(0)
        self.set_rc(3, 2000)
        self.delay_sim_time(5, reason="vehicle to travel north")
        self.set_rc(3, 1000)
        self.wait_groundspeed(0, 1)
        loc = self.get_location()
        self.progress("Driving East")
        self.set_rc(3, 2000)
        self.reach_heading_manual(90)
        self.set_rc(3, 2000)
        self.delay_sim_time(5, reason="vehicle to travel east")
        self.set_rc(3, 1000)

        self.progress("Entering smartrtl")
        self.change_mode("SMART_RTL")

        self.progress("Ensure we go via intermediate point")
        self.wait_distance_to_location(loc, 0, 5, timeout=60)

        self.progress("Ensure we get home")
        self.wait_distance_to_home(3, 7, timeout=30)

        self.disarm_vehicle()
