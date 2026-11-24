'''
Sub autotest: GripperMission

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class GripperMission(TestGroup):
    @testcase(bucket="Sub")
    def GripperMission(self):
        '''Test gripper mission items'''
        self.load_mission("sub-gripper-mission.txt")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.change_mode('AUTO')
        self.wait_waypoint(1, 2, max_dist_to_final_wp_m=5)
        self.wait_statustext("Gripper Grabbed", timeout=60)
        self.wait_waypoint(1, 4, max_dist_to_final_wp_m=5)
        self.wait_statustext("Gripper Released", timeout=60)
        self.wait_waypoint(1, 6, max_dist_to_final_wp_m=5)
        self.disarm_vehicle()
