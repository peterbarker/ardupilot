'''
Plane autotest: PreflightRebootComponent

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class PreflightRebootComponent(TestGroup):
    @testcase(bucket="PlaneTests1c")
    def PreflightRebootComponent(self):
        '''Ensure that PREFLIGHT_REBOOT commands sent to components don't reboot Autopilot'''
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            p1=1, # Reboot autopilot
            want_result=mavutil.mavlink.MAV_RESULT_DENIED,
            target_compid=mavutil.mavlink.MAV_COMP_ID_GIMBAL
        )
