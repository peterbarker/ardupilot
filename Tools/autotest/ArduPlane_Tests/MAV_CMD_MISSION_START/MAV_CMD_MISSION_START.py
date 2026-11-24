'''
Plane autotest: MAV_CMD_MISSION_START

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_MISSION_START(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def MAV_CMD_MISSION_START(self):
        '''test MAV_CMD_MISSION_START starts AUTO'''
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 800, 0, 0),
        ])
        for run_cmd in self.run_cmd, self.run_cmd_int:
            self.change_mode('LOITER')
            run_cmd(mavutil.mavlink.MAV_CMD_MISSION_START)
            self.wait_mode('AUTO')

        self.start_subtest("refused if AUTO may not be entered from the GCS")
        self.change_mode('LOITER')
        self.set_parameter("FLTMODE_GCSBLOCK", 1 << 9)  # AUTO
        for run_cmd in self.run_cmd, self.run_cmd_int:
            run_cmd(mavutil.mavlink.MAV_CMD_MISSION_START, want_result=mavutil.mavlink.MAV_RESULT_FAILED)
            self.assert_mode_is('LOITER')
