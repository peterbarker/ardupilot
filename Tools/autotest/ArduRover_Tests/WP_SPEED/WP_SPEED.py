'''
Rover autotest: WP_SPEED

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class WP_SPEED(TestGroup):
    @testcase(bucket="Rover")
    def WP_SPEED(self):
        '''ensure changing WP_SPEED during a mission works'''
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 300, 0, 0),
        ])
        start_speed_ms = self.get_parameter('WP_SPEED')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.change_mode('AUTO')

        self.wait_groundspeed(start_speed_ms-1, start_speed_ms+1, minimum_duration=10)

        for speed_ms in 1, 2, 3, 4, 5:
            self.set_parameter('WP_SPEED', speed_ms)
            self.wait_groundspeed(speed_ms-1, speed_ms+1, minimum_duration=10)
        self.do_RTL()
        self.disarm_vehicle()
