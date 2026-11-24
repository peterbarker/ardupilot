'''
Rover autotest: CrashCheck

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class CrashCheck(TestGroup):
    @testcase(bucket="Rover")
    def CrashCheck(self):
        """Test crash detection with FS_CRASH_CHECK 1 (hold) and 2 (hold+disarm)"""
        self.set_parameters({
            "CRASH_VEL_MIN": 60.0,    # unreachably high so any speed triggers
            "CRASH_TRAT_MIN": 360.0,  # same for turn rate
            "CRASH_TIMEOUT": 2.0,
            "CRASH_THR_MIN": 5.0,
        })
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 300, 0, 0),
        ])

        self.progress("Testing FS_CRASH_CHECK,1 (hold only)")
        self.set_parameter("FS_CRASH_CHECK", 1)
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.change_mode('AUTO')
        self.wait_statustext("Crash: Going to HOLD")
        self.wait_mode("HOLD")
        self.progress("Confirming still armed after hold-only crash")
        self.assert_armed()
        self.disarm_vehicle(force=True)
        self.progress("FS_CRASH_CHECK,1 (hold only) OK")

        self.progress("Testing FS_CRASH_CHECK,2 (hold + disarm)")
        self.set_parameter("FS_CRASH_CHECK", 2)
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.change_mode('AUTO')
        self.wait_statustext("Crash: Going to HOLD")
        self.wait_mode("HOLD")
        self.wait_disarmed()
        self.progress("FS_CRASH_CHECK,2 (hold + disarm) OK")
