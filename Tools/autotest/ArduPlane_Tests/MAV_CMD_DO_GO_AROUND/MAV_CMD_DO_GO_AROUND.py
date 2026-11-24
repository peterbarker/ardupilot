'''
Plane autotest: MAV_CMD_DO_GO_AROUND

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_DO_GO_AROUND(TestGroup):
    def _MAV_CMD_DO_GO_AROUND(self, command):
        self.load_mission("mission.txt")
        self.set_parameter("RTL_AUTOLAND", 3)
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.wait_current_waypoint(6)
        command(mavutil.mavlink.MAV_CMD_DO_GO_AROUND, p1=150)
        self.wait_current_waypoint(5)
        self.wait_altitude(135, 165, relative=True)
        self.wait_disarmed(timeout=300)

    @testcase(bucket="PlaneTests1a")
    def MAV_CMD_DO_GO_AROUND(self):
        '''test MAV_CMD_DO_GO_AROUND as a mavlink command'''
        self._MAV_CMD_DO_GO_AROUND(self.run_cmd)
        self._MAV_CMD_DO_GO_AROUND(self.run_cmd_int)
