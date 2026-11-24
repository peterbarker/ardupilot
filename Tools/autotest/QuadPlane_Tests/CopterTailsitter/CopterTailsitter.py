'''
QuadPlane autotest: CopterTailsitter

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class CopterTailsitter(TestGroup):
    @testcase(bucket="QuadPlaneTests1a")
    def CopterTailsitter(self):
        '''copter tailsitter test'''
        self.customise_SITL_commandline(
            [],
            model="quadplane-copter_tailsitter",
            wipe=True,
        )

        self.reboot_sitl()
        servo_under_test = 7  # We want to examine servo 7, assuming it's NOT covered by Q_TAILSIT_MOTMX.
        servo_string = f"SERVO{servo_under_test}_FUNCTION"
        self.progress('Assert that the servo is a quad motor')
        assert self.get_parameter(servo_string) >= 33 and self.get_parameter(servo_string) <= 36
        min_pwm = self.get_parameter("Q_M_PWM_MIN")

        self.wait_ready_to_arm()
        self.takeoff(60, mode='GUIDED')
        self.progress("Starting LOITER")
        self.change_mode("LOITER")
        self.context_collect("STATUSTEXT")
        # Wait for the transition to be done and no longer assisting.  A
        # settle-then-check is used here rather than polling for the servo
        # value: the VTOL motor output passes transiently through min_pwm
        # while it is still assisting.
        self.delay_sim_time(20, reason="transition to complete")
        servo_pwm = self.get_servo_channel_value(servo_under_test)
        if servo_pwm != min_pwm:
            raise NotAchievedException(f"The VTOL motor did not stop: {servo_pwm} != {min_pwm}")
        self.context_clear_collection("STATUSTEXT")
        self.progress("Starting QLAND")
        self.change_mode("QLAND")
        self.wait_statustext("Rangefinder engaged", check_context=True, timeout=60)
        self.wait_disarmed(timeout=100)
