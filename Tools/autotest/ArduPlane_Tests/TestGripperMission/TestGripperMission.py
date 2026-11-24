'''
Plane autotest: TestGripperMission

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class TestGripperMission(TestGroup):
    @testcase(bucket="PlaneTests1a")
    def TestGripperMission(self):
        '''Test Gripper mission items'''
        self.set_parameter("RTL_AUTOLAND", 1)
        self.load_mission("plane-gripper-mission.txt")
        self.set_current_waypoint(1)
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.wait_statustext("Gripper Grabbed", timeout=60)
        self.wait_statustext("Gripper Released", timeout=60)
        self.wait_statustext("Auto disarmed", timeout=60)
