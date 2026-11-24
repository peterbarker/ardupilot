'''
Rover autotest: RCModeChannelOptionConflict

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class RCModeChannelOptionConflict(TestGroup):
    @testcase(bucket="Rover")
    def RCModeChannelOptionConflict(self):
        '''ensure mode channel / RCn_OPTION conflict is a pre-arm failure'''
        self.wait_ready_to_arm()
        self.set_parameters({
            "MODE_CH": 8,
            "RC8_OPTION": 4,  # RTL
        })
        self.wait_not_ready_to_arm()
        self.assert_prearm_failure("Mode channel and RC8_OPTION conflict")

        self.progress("Conflict still blocks arming with PARAMETERS check skipped")
        self.set_parameter("ARMING_SKIPCHK", 1 << 5)
        self.assert_prearm_failure("Mode channel and RC8_OPTION conflict")
        self.assert_arm_failure("Mode channel and RC8_OPTION conflict")
        self.set_parameter("ARMING_SKIPCHK", 0)

        self.progress("Conflict not reported with RC check skipped")
        self.set_parameter("ARMING_SKIPCHK", 1 << 6)
        self.wait_ready_to_arm()
        self.set_parameter("ARMING_SKIPCHK", 0)
        self.wait_not_ready_to_arm()

        self.progress("Conflict reported without RC input")
        self.set_parameter("SIM_RC_FAIL", 1)
        self.wait_sensor_state(mavutil.mavlink.MAV_SYS_STATUS_SENSOR_RC_RECEIVER, healthy=False)
        self.assert_prearm_failure("Mode channel and RC8_OPTION conflict",
                                   other_prearm_failures_fatal=False)
        self.set_parameter("SIM_RC_FAIL", 0)
        self.wait_sensor_state(mavutil.mavlink.MAV_SYS_STATUS_SENSOR_RC_RECEIVER, healthy=True)

        self.set_parameter("RC8_OPTION", 0)
        self.wait_ready_to_arm()
