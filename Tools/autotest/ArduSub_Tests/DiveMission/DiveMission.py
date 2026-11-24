'''
Sub autotest: DiveMission

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class DiveMission(TestGroup):
    @testcase(bucket="Sub")
    def DiveMission(self):
        '''Dive mission'''
        filename = "sub_mission.txt"
        self.progress(f"Executing mission {filename}")
        self.load_mission(filename)
        self.set_rc_default()

        self.arm_vehicle()

        self.change_mode('AUTO')

        self.wait_waypoint(1, 5, max_dist_to_final_wp_m=5)

        self.disarm_vehicle()

        self.progress("Mission OK")
