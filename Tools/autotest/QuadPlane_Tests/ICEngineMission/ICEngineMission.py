'''
QuadPlane autotest: ICEngineMission

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class ICEngineMission(TestGroup):
    @testcase(bucket="QuadPlaneTests1a")
    def ICEngineMission(self):
        '''Test ICE Engine Mission support'''
        rc_engine_start_chan = 11
        self.setup_ICEngine_vehicle()

        self.load_mission("mission.txt")
        self.wait_ready_to_arm()
        self.set_rc(rc_engine_start_chan, 2000)
        self.arm_vehicle()
        self.change_mode('AUTO')
        self.wait_disarmed(timeout=300)
