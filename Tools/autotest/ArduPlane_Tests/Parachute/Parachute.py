'''
Plane autotest: Parachute

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class Parachute(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def Parachute(self):
        '''Test Parachute'''
        self.set_rc(9, 1000)
        self.setup_simulated_parachute()

        self.load_mission("plane-parachute-mission.txt")
        self.set_current_waypoint(1)
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.wait_statustext("BANG", timeout=60)
        self.disarm_vehicle(force=True)
        self.reboot_sitl()
