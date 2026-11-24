'''
Plane autotest: ParachuteSinkRate

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class ParachuteSinkRate(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def ParachuteSinkRate(self):
        '''Test Parachute (SinkRate triggering)'''
        self.set_rc(9, 1000)
        self.setup_simulated_parachute({"CHUTE_CRT_SINK": 9})

        self.progress("Takeoff")
        self.takeoff(alt=300)

        self.progress("Diving")
        self.set_rc(2, 2000)
        self.wait_statustext("BANG", timeout=60)

        self.disarm_vehicle(force=True)
        self.reboot_sitl()
