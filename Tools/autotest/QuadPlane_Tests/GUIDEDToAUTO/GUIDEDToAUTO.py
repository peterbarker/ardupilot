'''
QuadPlane autotest: GUIDEDToAUTO

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class GUIDEDToAUTO(TestGroup):
    @testcase(bucket="QuadPlaneTests1c")
    def GUIDEDToAUTO(self):
        '''Test using GUIDED mode for takeoff before shifting to auto'''
        self.install_terrain_handlers_context()  # mission.txt uses terrain-frame

        self.load_mission("mission.txt")
        self.takeoff(30, mode='GUIDED')

        # extra checks would go here
        self.assert_not_receiving_message('CAMERA_FEEDBACK')

        self.change_mode('AUTO')
        self.wait_current_waypoint(3)
        self.change_mode('QRTL')
        self.wait_disarmed(timeout=240)
