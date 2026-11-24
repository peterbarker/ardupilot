'''
Helicopter autotest: MountFailsafeAction

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class MountFailsafeAction(TestGroup):
    @testcase(bucket="Helicopter")
    def MountFailsafeAction(self):
        """Fly Mount Failsafe action"""
        self.context_push()

        self.progress("Setting up servo mount")
        roll_servo = 12
        pitch_servo = 11
        yaw_servo = 10
        open_servo = 9
        roll_limit = 50
        self.set_parameters({
            "MNT1_TYPE": 1,
            "SERVO%u_MIN" % roll_servo: 1000,
            "SERVO%u_MAX" % roll_servo: 2000,
            "SERVO%u_FUNCTION" % yaw_servo: 6,  # yaw
            "SERVO%u_FUNCTION" % pitch_servo: 7,  # roll
            "SERVO%u_FUNCTION" % roll_servo: 8,  # pitch
            "SERVO%u_FUNCTION" % open_servo: 9,  # mount open
            "MNT1_OPTIONS": 2,  # retract
            "MNT1_DEFLT_MODE": 3,  # RC targeting
            "MNT1_ROLL_MIN": -roll_limit,
            "MNT1_ROLL_MAX": roll_limit,
        })

        self.reboot_sitl()

        retract_roll = 25.0
        self.set_parameter("MNT1_NEUTRAL_X", retract_roll)
        self.progress("Killing RC")
        self.set_parameter("SIM_RC_FAIL", 2)
        want_servo_channel_value = int(1500 + 500*retract_roll/roll_limit)
        self.wait_servo_channel_value(roll_servo, want_servo_channel_value, epsilon=1, timeout=12)

        self.progress("Resurrecting RC")
        self.set_parameter("SIM_RC_FAIL", 0)
        self.wait_servo_channel_value(roll_servo, 1500)

        self.context_pop()

        self.reboot_sitl()
