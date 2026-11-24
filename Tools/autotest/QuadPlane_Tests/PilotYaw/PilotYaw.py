'''
QuadPlane autotest: PilotYaw

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class PilotYaw(TestGroup):
    @testcase(bucket="QuadPlaneTests1c")
    def PilotYaw(self):
        '''Test pilot yaw in various modes'''
        self.takeoff(10, mode="QLOITER")
        self.set_parameter("STICK_MIXING", 0)
        self.set_rc(4, 1700)
        for mode in "QLOITER", "QHOVER":
            self.wait_heading(45)
            self.wait_heading(90)
            self.wait_heading(180)
            self.wait_heading(275)
        self.set_rc(4, 1500)
        self.do_RTL()
