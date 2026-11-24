'''
Copter autotest: BodyFrameOdom

AP_FLAKE8_CLEAN
'''

import time

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class BodyFrameOdom(TestGroup):
    @testcase(bucket="CopterTests1e")
    def BodyFrameOdom(self):
        """Disable GPS navigation, enable input of VISION_POSITION_DELTA."""

        if self.get_parameter("AHRS_EKF_TYPE") != 3:
            # only tested on this EKF
            return

        self.customise_SITL_commandline(["--serial5=sim:vicon:"])

        if self.current_onboard_log_contains_message("XKFD"):
            raise NotAchievedException("Found unexpected XKFD message")

        # scribble down a location we can set origin to:
        self.progress("Waiting for location")
        self.change_mode('LOITER')
        self.wait_ready_to_arm()

        old_pos = self.assert_receive_message('GLOBAL_POSITION_INT')
        print("old_pos=%s" % str(old_pos))

        # configure EKF to use external nav instead of GPS
        self.set_parameters({
            "EK3_SRC1_POSXY": 6,
            "EK3_SRC1_VELXY": 6,
            "EK3_SRC1_POSZ": 6,
            "EK3_SRC1_VELZ": 6,
            "GPS1_TYPE": 0,
            "VISO_TYPE": 1,
            "SERIAL5_PROTOCOL": 1,
            "SIM_VICON_TMASK": 8,  # send VISION_POSITION_DELTA
        })
        self.reboot_sitl()
        # without a GPS or some sort of external prompting, AP
        # doesn't send system_time messages.  So prompt it:
        self.mav.mav.system_time_send(int(time.time() * 1000000), 0)
        self.set_origin(old_pos)

        self.takeoff(altitude_min=5, mode='ALT_HOLD', require_absolute=False, takeoff_throttle=1800)
        self.change_mode('LAND')
        # TODO: something more elaborate here - EKF will only aid
        # relative position
        self.wait_disarmed()
        if not self.current_onboard_log_contains_message("XKFD"):
            raise NotAchievedException("Did not find expected XKFD message")
