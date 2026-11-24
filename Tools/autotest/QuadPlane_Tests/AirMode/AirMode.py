'''
QuadPlane autotest: AirMode

AP_FLAKE8_CLEAN
'''

import math
import operator

from vehicle_test_suite import NotAchievedException
from vehicle_test_suite import PreconditionFailedException

from test_discovery import TestGroup
from test_discovery import testcase


class AirMode(TestGroup):
    @testcase(bucket="QuadPlaneTests1a")
    def AirMode(self):
        """Check that plane.air_mode turns on and off as required"""
        self.progress("########## Testing AirMode operation")
        self.set_parameter("AHRS_EKF_TYPE", 10)
        self.change_mode('QSTABILIZE')
        self.wait_ready_to_arm()

        """
        SPIN_ARM and SPIN_MIN default to 0.10 and 0.15
        when armed with zero throttle in AirMode, motor PWM should be at SPIN_MIN
        If AirMode is off, motor PWM will drop to SPIN_ARM
        """

        self.progress("Verify that SERVO5 is Motor1 (default)")
        motor1_servo_function_lp = 33
        if (self.get_parameter('SERVO5_FUNCTION') != motor1_servo_function_lp):
            raise PreconditionFailedException("SERVO5_FUNCTION not %d" % motor1_servo_function_lp)

        self.progress("Verify that flightmode channel is 5 (default)")
        default_fltmode_ch = 5
        if (self.get_parameter("FLTMODE_CH") != default_fltmode_ch):
            raise PreconditionFailedException("FLTMODE_CH not %d" % default_fltmode_ch)

        """When disarmed, motor PWM will drop to min_pwm"""
        min_pwm = self.get_parameter("Q_M_PWM_MIN")

        self.progress("Verify Motor1 is at min_pwm when disarmed")
        self.wait_servo_channel_value(5, min_pwm, comparator=operator.eq)

        armdisarm_option = 154
        arm_ch = 8
        self.set_parameter("RC%d_OPTION" % arm_ch, armdisarm_option)
        self.progress("Configured RC%d as ARMDISARM switch" % arm_ch)

        """arm with GCS, record Motor1 SPIN_ARM PWM output and disarm"""
        spool_delay = self.get_parameter("Q_M_SPOOL_TIME") + 0.25
        self.zero_throttle()
        self.arm_vehicle()
        self.progress("Waiting for Motor1 to spool up to SPIN_ARM")
        self.delay_sim_time(spool_delay, reason="motor spool-up")
        spin_arm_pwm = self.wait_servo_channel_value(5, min_pwm, comparator=operator.gt)
        self.progress("spin_arm_pwm: %d" % spin_arm_pwm)
        self.disarm_vehicle()

        """arm with switch, record Motor1 SPIN_MIN PWM output and disarm"""
        self.set_rc(8, 2000)
        self.delay_sim_time(spool_delay, reason="motor spool-up")
        self.progress("Waiting for Motor1 to spool up to SPIN_MIN")
        spin_min_pwm = self.wait_servo_channel_value(5, spin_arm_pwm, comparator=operator.gt)
        self.progress("spin_min_pwm: %d" % spin_min_pwm)
        self.set_rc(8, 1000)

        if (spin_arm_pwm >= spin_min_pwm):
            raise PreconditionFailedException("SPIN_MIN pwm not greater than SPIN_ARM pwm")

        self.start_subtest("Test auxswitch arming with AirMode Switch")
        for mode in ('QSTABILIZE', 'QACRO'):
            """verify that arming with switch results in higher PWM output"""
            self.progress("Testing %s mode" % mode)
            self.change_mode(mode)
            self.zero_throttle()
            self.progress("Arming with switch at zero throttle")
            self.arm_motors_with_switch(arm_ch)
            self.progress("Waiting for Motor1 to speed up")
            self.wait_servo_channel_value(5, spin_min_pwm, comparator=operator.ge)

            self.progress("Verify that rudder disarm is disabled")
            try:
                self.disarm_motors_with_rc_input()
            except NotAchievedException:
                pass
            if not self.armed():
                raise NotAchievedException("Rudder disarm not disabled")

            self.progress("Disarming with switch")
            self.disarm_motors_with_switch(arm_ch)
            self.progress("Waiting for Motor1 to stop")
            self.wait_servo_channel_value(5, min_pwm, comparator=operator.le)
            self.wait_ready_to_arm()

        self.start_subtest("Verify that arming with switch does not spin motors in other modes")
        # disable compass magnetic field arming check that is triggered by the simulated lean of vehicle
        # this is required because adjusting the AHRS_TRIM values only affects the IMU and not external compasses
        arming_magthresh = self.get_parameter("ARMING_MAGTHRESH")
        self.set_parameter("ARMING_MAGTHRESH", 0)
        # introduce a large attitude error to verify that stabilization is not active
        ahrs_trim_x = self.get_parameter("AHRS_TRIM_X")
        self.set_parameter("AHRS_TRIM_X", math.radians(-60))
        self.wait_roll(60, 1)
        # test all modes except QSTABILIZE, QACRO, AUTO and QAUTOTUNE and QLAND and QRTL
        # QRTL and QLAND aren't tested because we can't arm in that mode
        for mode in (
                'ACRO',
                'AUTOTUNE',
                'AVOID_ADSB',
                'CIRCLE',
                'CRUISE',
                'FBWA',
                'FBWB',
                'GUIDED',
                'LOITER',
                'QHOVER',
                'QLOITER',
                'STABILIZE',
                'TRAINING',
        ):
            self.progress("Testing %s mode" % mode)
            self.change_mode(mode)
            self.zero_throttle()
            self.progress("Arming with switch at zero throttle")
            self.arm_motors_with_switch(arm_ch)
            self.progress("Waiting for Motor1 to (not) speed up")
            self.delay_sim_time(spool_delay, reason="motor spool-up period")
            self.wait_servo_channel_value(5, spin_arm_pwm, comparator=operator.le)
            self.wait_servo_channel_value(6, spin_arm_pwm, comparator=operator.le)
            self.wait_servo_channel_value(7, spin_arm_pwm, comparator=operator.le)
            self.wait_servo_channel_value(8, spin_arm_pwm, comparator=operator.le)

            self.progress("Disarming with switch")
            self.disarm_motors_with_switch(arm_ch)
            self.progress("Waiting for Motor1 to stop")
            self.wait_servo_channel_value(5, min_pwm, comparator=operator.le)
            self.wait_ready_to_arm()
        # remove attitude error and reinstance compass arming check
        self.set_parameter("AHRS_TRIM_X", ahrs_trim_x)
        self.set_parameter("ARMING_MAGTHRESH", arming_magthresh)

        self.start_subtest("verify that AIRMODE auxswitch turns airmode on/off while armed")
        """set  RC7_OPTION to AIRMODE"""
        option_airmode = 84
        self.set_parameter("RC7_OPTION", option_airmode)

        for mode in ('QSTABILIZE', 'QACRO'):
            self.progress("Testing %s mode" % mode)
            self.change_mode(mode)
            self.zero_throttle()
            self.progress("Arming with GCS at zero throttle")
            self.arm_vehicle()

            self.progress("Turn airmode on with auxswitch")
            self.set_rc(7, 2000)
            self.progress("Waiting for Motor1 to speed up")
            self.wait_servo_channel_value(5, spin_min_pwm, comparator=operator.ge)

            self.progress("Turn airmode off with auxswitch")
            self.set_rc(7, 1000)
            self.progress("Waiting for Motor1 to slow down")
            self.wait_servo_channel_value(5, spin_arm_pwm, comparator=operator.le)
            self.disarm_vehicle()
            self.wait_ready_to_arm()

        self.start_subtest("Test GCS arming")
        for mode in ('QSTABILIZE', 'QACRO'):
            self.progress("Testing %s mode" % mode)
            self.change_mode(mode)
            self.zero_throttle()
            self.progress("Arming with GCS at zero throttle")
            self.arm_vehicle()

            self.progress("Turn airmode on with auxswitch")
            self.set_rc(7, 2000)
            self.progress("Waiting for Motor1 to speed up")
            self.wait_servo_channel_value(5, spin_min_pwm, comparator=operator.ge)

            self.disarm_vehicle_expect_fail()
            self.arm_vehicle()

            self.progress("Verify that airmode is still on")
            self.wait_servo_channel_value(5, spin_min_pwm, comparator=operator.ge)
            self.disarm_vehicle(force=True)
            self.wait_ready_to_arm()
