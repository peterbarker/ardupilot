'''
Rover autotest: GripperMission

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class GripperMission(TestGroup):
    @testcase(bucket="Rover")
    def GripperMission(self):
        '''Test Gripper Mission Items'''
        self.load_mission("rover-gripper-mission.txt")
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.context_collect('STATUSTEXT')
        self.wait_statustext("Gripper Grabbed", timeout=60, check_context=True)
        self.wait_statustext("Gripper Released", timeout=60, check_context=True)
        self.wait_statustext("Mission Complete", timeout=60, check_context=True)
        self.disarm_vehicle()
