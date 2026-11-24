'''
Copter autotest: MISSION_START

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class MISSION_START(TestGroup):
    @testcase(bucket="CopterTests1c")
    def MISSION_START(self):
        '''test mavlink command MAV_CMD_MISSION_START'''
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 200),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        for command in self.run_cmd, self.run_cmd_int:
            self.change_mode('LOITER')
            self.set_current_waypoint(1)
            self.wait_ready_to_arm()
            self.arm_vehicle()
            self.change_mode('AUTO')
            command(mavutil.mavlink.MAV_CMD_MISSION_START)
            self.wait_altitude(20, 1000, relative=True)
            self.change_mode('RTL')
            self.wait_disarmed()
