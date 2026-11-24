'''
Copter autotest: BodyFrameOdomDelay

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class BodyFrameOdomDelay(TestGroup):
    @testcase(bucket="CopterTests1e")
    def BodyFrameOdomDelay(self):
        """EKF3 relative aiding from VISION_POSITION_DELTA must not depend on VISO_DELAY_MS."""

        if self.get_parameter("AHRS_EKF_TYPE") != 3:
            # only tested on this EKF
            return

        self.customise_SITL_commandline(["--serial5=sim:vicon:"])

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

        required_flags = (mavutil.mavlink.ESTIMATOR_VELOCITY_HORIZ |
                          mavutil.mavlink.ESTIMATOR_POS_HORIZ_REL |
                          mavutil.mavlink.ESTIMATOR_PRED_POS_HORIZ_REL)
        error_bits = mavutil.mavlink.ESTIMATOR_CONST_POS_MODE

        for delay_ms in 0, 40, 200, 250:
            self.start_subtest(f"VISO_DELAY_MS={delay_ms}")
            # the delay sizes the EKF buffers at startup, so reboot after changing it
            self.set_parameter("VISO_DELAY_MS", delay_ms)
            self.reboot_sitl()
            # the EKF must start relative aiding from the body-frame odometry
            # and leave constant-position mode, whatever the configured delay
            self.wait_ekf_flags(required_flags, error_bits, timeout=60, minimum_duration=5)
            if not self.current_onboard_log_contains_message("XKFD"):
                raise NotAchievedException(f"Did not find expected XKFD message at VISO_DELAY_MS={delay_ms}")
