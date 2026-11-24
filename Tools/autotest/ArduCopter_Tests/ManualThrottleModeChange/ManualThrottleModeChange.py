'''
Copter autotest: ManualThrottleModeChange

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class ManualThrottleModeChange(TestGroup):
    @testcase(bucket="CopterTests1e")
    def ManualThrottleModeChange(self):
        '''Check manual throttle mode changes denied on high throttle'''
        self.set_parameter("FS_GCS_ENABLE", 0) # avoid GUIDED instant disarm
        self.change_mode("STABILIZE")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.change_mode("ACRO")
        self.change_mode("STABILIZE")
        self.change_mode("GUIDED")
        self.set_rc(3, 1700)
        # should not take off in guided:
        self.wait_altitude(
            altitude_min=-1,
            altitude_max=0.2,
            relative=True,
            minimum_duration=5,
            timeout=6,
        )
        self.run_cmd_do_set_mode(
            "ACRO",
            want_result=mavutil.mavlink.MAV_RESULT_FAILED)
        self.run_cmd_do_set_mode(
            "STABILIZE",
            want_result=mavutil.mavlink.MAV_RESULT_FAILED)
        self.run_cmd_do_set_mode(
            "DRIFT",
            want_result=mavutil.mavlink.MAV_RESULT_FAILED)
        self.progress("Check setting an invalid mode")
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            p1=mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            p2=126,
            want_result=mavutil.mavlink.MAV_RESULT_FAILED,
            timeout=1,
        )
        self.set_rc(3, 1000)
        self.run_cmd_do_set_mode("ACRO")
        self.wait_disarmed()
