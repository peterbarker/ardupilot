'''
QuadPlane autotest: Mission

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class Mission(TestGroup):
    @testcase(bucket="QuadPlaneTests1c")
    def Mission(self):
        '''fly the OBC 2016 mission in Dalby'''
        self.load_mission("Dalby-OBC2016.txt")
        self.load_fence("Dalby-OBC2016-fence.txt")
        if self.mavproxy is not None:
            self.mavproxy.send('wp list\n')
        self.install_terrain_handlers_context()
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.change_mode('AUTO')
        self.wait_waypoint(1, 19, max_dist_to_final_wp_m=60, timeout=1200)

        self.wait_disarmed(timeout=120) # give quadplane a long time to land
        # wait for blood sample here
        self.set_current_waypoint(20)
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.wait_waypoint(20, 34, max_dist_to_final_wp_m=60, timeout=1200)

        self.wait_disarmed(timeout=120) # give quadplane a long time to land
        self.progress("Mission OK")
