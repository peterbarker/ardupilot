'''
Copter autotest: WP_SPEED_UP

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class WP_SPEED_UP(TestGroup):
    @testcase(bucket="CopterTests2a")
    def WP_SPEED_UP(self):
        '''Change speed (up) during mission'''

        items = []

        # 1 waypoint a long way up
        items.append((mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 20000),)

        items.append((mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0))

        self.upload_simple_relhome_mission(items)

        start_speed_ms = self.get_parameter('WP_SPD_UP')

        minimum_duration = 5

        self.takeoff(20, mode='GUIDED')
        self.change_mode('AUTO')
        self.wait_climbrate(start_speed_ms-1, start_speed_ms+1, minimum_duration=minimum_duration)

        for speed_ms in 7, 8, 7, 8, 6, 2:
            self.set_parameter('WP_SPD_UP', speed_ms)
            self.wait_climbrate(speed_ms-1, speed_ms+1, minimum_duration=minimum_duration)
        self.do_RTL(timeout=240)
