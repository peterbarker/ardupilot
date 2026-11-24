'''
Copter autotest: TestGripperMission

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class TestGripperMission(TestGroup):
    @testcase(bucket="CopterTests2d")
    def TestGripperMission(self):
        '''Test Gripper mission items'''
        num_wp = self.load_mission("copter-gripper-mission.txt")
        self.change_mode('LOITER')
        self.wait_ready_to_arm()
        self.assert_vehicle_location_is_at_startup_location()
        self.arm_vehicle()
        self.change_mode('AUTO')
        self.set_rc(3, 1500)
        self.wait_statustext("Gripper Grabbed", timeout=60)
        self.wait_statustext("Gripper Released", timeout=60)
        self.wait_waypoint(num_wp-1, num_wp-1)
        self.wait_disarmed()
