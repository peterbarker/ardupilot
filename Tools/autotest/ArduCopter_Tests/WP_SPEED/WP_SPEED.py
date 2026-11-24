'''
Copter autotest: WP_SPEED

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class WP_SPEED(TestGroup):
    @testcase(bucket="CopterTests1e")
    def WP_SPEED(self):
        '''ensure resetting WP_SPD during a mission works'''

        loc = self.poll_home_position()
        alt = 20
        loc.alt = alt
        items = []

        # 100 waypoints in a line, 10m apart in a northerly direction
        #        for i in range(1, 100):
        #            items.append((mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, i*10, 0, alt))

        # 1 waypoint a long way away
        items.append((mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2000, 0, alt),)

        items.append((mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0))

        self.upload_simple_relhome_mission(items)

        start_speed_ms = self.get_parameter('WP_SPD')

        self.takeoff(20, mode='GUIDED')
        self.change_mode('AUTO')
        self.wait_groundspeed(start_speed_ms-1, start_speed_ms+1, minimum_duration=10)

        for speed_ms in 7, 8, 7, 8, 9, 10, 11, 7:
            self.set_parameter('WP_SPD', speed_ms)
            self.wait_groundspeed(speed_ms-1, speed_ms+1, minimum_duration=10)
        self.do_RTL()
