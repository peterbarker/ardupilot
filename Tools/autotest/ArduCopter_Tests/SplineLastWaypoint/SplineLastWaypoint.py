'''
Copter autotest: SplineLastWaypoint

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class SplineLastWaypoint(TestGroup):
    @testcase(bucket="CopterTests1c")
    def SplineLastWaypoint(self):
        '''Test Spline as last waypoint'''
        self.load_mission("copter-spline-last-waypoint.txt")
        self.change_mode('LOITER')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.change_mode('AUTO')
        self.set_rc(3, 1500)
        self.wait_altitude(10, 3000, relative=True)
        self.do_RTL()
