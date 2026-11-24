'''
Helicopter autotest: SplineWaypoint

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class SplineWaypoint(TestGroup):
    @testcase(bucket="Helicopter")
    def SplineWaypoint(self, timeout=600):
        """ensure basic spline functionality works"""
        self.load_mission("copter_spline_mission.txt", strict=False)
        self.change_mode("LOITER")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.progress("Raising rotor speed")
        self.set_rc(8, 2000)
        self.delay_sim_time(20, reason="rotor to reach speed")
        self.change_mode("AUTO")
        self.set_rc(3, 1500)
        self.wait_disarmed(timeout=600)
        self.progress("Lowering rotor speed")
        self.set_rc(8, 1000)
