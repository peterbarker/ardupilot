'''
QuadPlane autotest: VTOLLandSpiral

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class VTOLLandSpiral(TestGroup):
    @testcase(bucket="QuadPlaneTests1a")
    def VTOLLandSpiral(self):
        '''check spiral-to-alt option for landing'''
        self.fly_mission('mission.txt')
        self.set_parameter('WP_LOITER_RAD', -self.get_parameter('WP_LOITER_RAD'))
        self.set_current_waypoint(0, check_afterwards=False)
        self.fly_mission('mission.txt')
