'''
Plane autotest: RudderArmedTakeoffRequiresNeutralThrottle

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class RudderArmedTakeoffRequiresNeutralThrottle(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def RudderArmedTakeoffRequiresNeutralThrottle(self):
        '''auto-takeoff should not occur while rudder continues to be held over'''
        self.change_mode('TAKEOFF')
        self.wait_ready_to_arm()
        self.set_rc(4, 1000)
        self.wait_armed()
        self.wait_groundspeed(0, 1, minimum_duration=10)
        self.set_rc(4, 1500)
        self.wait_groundspeed(5, 100)
        self.fly_home_land_and_disarm()
