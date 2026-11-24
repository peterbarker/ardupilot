'''
Copter autotest: MAV_CMD_NAV_TAKEOFF_no_location

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_NAV_TAKEOFF_no_location(TestGroup):
    @testcase(bucket="CopterTests2a")
    def MAV_CMD_NAV_TAKEOFF_no_location(self):
        '''make sure nav_takeoff works in alt_hold when option forced'''
        self.set_parameter('GPS1_TYPE', 0)
        self.reboot_sitl()
        self.change_mode('ALT_HOLD')
        self.wait_prearm_sys_status_healthy()
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            p7=10,
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            want_result=mavutil.mavlink.MAV_RESULT_FAILED,
        )
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            p3=1,
            p7=10,
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            want_result=mavutil.mavlink.MAV_RESULT_FAILED,
        )
        self.disarm_vehicle(force=True)
