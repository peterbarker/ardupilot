'''
Rover autotest: RTL_SPEED

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class RTL_SPEED(TestGroup):
    @testcase(bucket="Rover")
    def RTL_SPEED(self, timeout=120):
        '''Test RTL_SPEED is honoured'''

        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 300, 0, 0),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 1000, 0, 0),
        ])

        self.wait_ready_to_arm()
        self.arm_vehicle()

        self.change_mode('AUTO')
        self.wait_current_waypoint(2, timeout=120)
        for speed in 1, 5.5, 1.5, 7.5:
            self.set_parameter("RTL_SPEED", speed)
            self.change_mode('RTL')
            self.wait_groundspeed(speed-0.1, speed+0.1, minimum_duration=10)
            self.change_mode('HOLD')
        self.do_RTL()
        self.disarm_vehicle()
