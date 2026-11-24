'''
Rover autotest: ThrottleFailsafe

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class ThrottleFailsafe(TestGroup):
    @testcase(bucket="Rover")
    def ThrottleFailsafe(self):
        """Trigger throttle failsafes"""
        self.progress("Testing throttle failsafe")
        # set failsafe to Loiter_or_Hold
        self.set_parameters({
            "FS_THR_ENABLE": 1,
            "FS_ACTION": 6,
        })
        self.change_mode("MANUAL")
        self.wait_ready_to_arm()
        self.arm_vehicle()

        self.progress("Testing failsafe to loiter")
        self.set_parameter("SIM_RC_FAIL", 1)  # no-pulses failsafe
        self.wait_mode("LOITER", timeout=10)

        # restore RC and prepare to test hold fallback
        self.set_parameter("SIM_RC_FAIL", 0)
        self.set_parameter("SIM_GPS1_ENABLE", 0)
        self.change_mode("MANUAL")

        self.progress("Testing failsafe to hold")
        self.set_parameter("SIM_RC_FAIL", 1)
        self.wait_mode("HOLD", timeout=10)
        self.disarm_vehicle()
        self.progress("Loiter or Hold as throttle failsafe OK")
