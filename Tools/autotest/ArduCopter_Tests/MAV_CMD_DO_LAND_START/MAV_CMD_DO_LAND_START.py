'''
Copter autotest: MAV_CMD_DO_LAND_START

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_DO_LAND_START(TestGroup):
    def _MAV_CMD_DO_LAND_START(self, run_cmd):
        alt = 5
        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, alt),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 200, 0, alt),
            (mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0),
            (mavutil.mavlink.MAV_CMD_DO_LAND_START, 0, 0, alt),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 200, 2000, alt),
            (mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0),
        ])

        self.wait_current_waypoint(2)
        run_cmd(mavutil.mavlink.MAV_CMD_DO_LAND_START)
        self.wait_current_waypoint(5)
        # we pretend to be in RTL mode while doing this:
        self.wait_mode("AUTO_RTL")
        self.do_RTL()

    @testcase(bucket="CopterTests2c")
    def MAV_CMD_DO_LAND_START(self):
        '''test handling of mavlink-received MAV_CMD_DO_LAND_START command'''
        self._MAV_CMD_DO_LAND_START(self.run_cmd)
        self.zero_throttle()
        self._MAV_CMD_DO_LAND_START(self.run_cmd_int)
