'''
Plane autotest: DO_PARACHUTE

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class DO_PARACHUTE(TestGroup):
    @testcase(bucket="PlaneTests1c")
    def DO_PARACHUTE(self):
        '''test triggering parachute via mavlink'''
        self.setup_simulated_parachute({"FS_LONG_ACTN": 3})
        for command in self.run_cmd, self.run_cmd_int:
            # We release the parachute sitting on the ground, which the
            # vehicle permits only while it has never flown:
            # parachute_manual_release() skips its "Too low" check on
            # last_flying_ms being zero, and that is sticky for the life
            # of the boot.  Any test which flew before us leaves it set
            # and the release is refused with MAV_RESULT_FAILED - so
            # start from a fresh boot rather than only leaving one
            # behind for the iteration which follows.
            self.reboot_sitl()
            self.wait_servo_channel_value(9, 1100)
            self.wait_ready_to_arm()
            self.arm_vehicle()
            command(
                mavutil.mavlink.MAV_CMD_DO_PARACHUTE,
                p1=mavutil.mavlink.PARACHUTE_RELEASE,
            )
            self.wait_servo_channel_value(9, 1300)
            self.disarm_vehicle()
