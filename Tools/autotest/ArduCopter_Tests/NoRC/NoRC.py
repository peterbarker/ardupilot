'''
Copter autotest: NoRC

AP_FLAKE8_CLEAN
'''

import math

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class NoRC(TestGroup):
    @testcase(bucket="CopterTests1a")
    def NoRC(self):
        '''check what happens if we fly with no RC'''
        self.set_parameters({
            "SIM_RC_FAIL": 1,
            "FS_THR_ENABLE": 0,
            "AUTO_OPTIONS": 3,
        })
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.reboot_sitl()

        # add a listener that verifies yaw rate remains zero:
        def verify_yaw(mav, m):
            if m.get_type() != 'ATTITUDE':
                return
            thresh_degs = 10
            current_degs = math.degrees(abs(m.yawspeed))
            if current_degs > thresh_degs:
                raise NotAchievedException(
                    f"Excessive yaw: {current_degs} deg/s > {thresh_degs} deg/s"
                )
        self.install_message_hook_context(verify_yaw)

        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.wait_disarmed()
