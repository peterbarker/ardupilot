'''
Copter autotest: MAV_CMD_MISSION_START_p1_p2

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_MISSION_START_p1_p2(TestGroup):
    @testcase(bucket="CopterTests1d")
    def MAV_CMD_MISSION_START_p1_p2(self):
        '''make sure we deny MAV_CMD_MISSION_START if either p1 or p2 non-zero'''
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),
        ])
        self.set_parameters({
            "AUTO_OPTIONS": 3,
        })
        self.change_mode('AUTO')
        self.wait_ready_to_arm()

        self.run_cmd(
            mavutil.mavlink.MAV_CMD_MISSION_START,
            p1=1,
            want_result=mavutil.mavlink.MAV_RESULT_DENIED,
        )

        self.run_cmd(
            mavutil.mavlink.MAV_CMD_MISSION_START,
            p2=1,
            want_result=mavutil.mavlink.MAV_RESULT_DENIED,
        )

        self.run_cmd(
            mavutil.mavlink.MAV_CMD_MISSION_START,
            p1=1,
            p2=1,
            want_result=mavutil.mavlink.MAV_RESULT_DENIED,
        )
