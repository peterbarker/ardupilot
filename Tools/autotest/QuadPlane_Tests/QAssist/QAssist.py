'''
QuadPlane autotest: QAssist

AP_FLAKE8_CLEAN
'''

import operator

from vehicle_test_suite import AltFrame

from test_discovery import TestGroup
from test_discovery import testcase


class QAssist(TestGroup):
    @testcase(bucket="QuadPlaneTests1a")
    def QAssist(self):
        '''QuadPlane Assist tests'''
        self.takeoff(50, mode="QHOVER", timeout=120)
        self.set_rc(3, 1800)
        self.change_mode("FBWA")

        # disable stall prevention so roll angle is not limited
        self.set_parameter("STALL_PREVENTION", 0)

        thr_min_pwm = self.get_parameter("Q_M_PWM_MIN")
        lim_roll_deg = self.get_parameter("ROLL_LIMIT_DEG")
        lim_pitch_down_deg = self.get_parameter("PTCH_LIM_MIN_DEG")
        lim_pitch_up_deg = self.get_parameter("PTCH_LIM_MAX_DEG")
        self.progress("Waiting for motors to stop (transition completion)")
        self.wait_servo_channel_value(5,
                                      thr_min_pwm,
                                      timeout=30,
                                      comparator=operator.eq)
        self.delay_sim_time(5, reason="motors to stop after transition")
        self.wait_servo_channel_value(5,
                                      thr_min_pwm,
                                      timeout=30,
                                      comparator=operator.eq)
        self.progress("Stopping forward motor to kill airspeed below limit")
        self.set_rc(3, 1000)
        self.progress("Waiting for qassist to kick in")
        self.wait_servo_channel_value(5, 1400, timeout=30, comparator=operator.gt)
        self.progress("Move forward again, check qassist stops")
        self.set_rc(3, 1800)
        self.progress("Checking qassist stops")
        self.wait_servo_channel_value(5,
                                      thr_min_pwm,
                                      timeout=30,
                                      comparator=operator.eq)
        self.set_rc(3, 1300)

        # Disable speed assist and alt assist during angle assist tests
        self.set_parameters({
            "Q_ASSIST_SPEED": 1,
            "Q_ASSIST_ALT": 0,
        })

        self.start_subtest("Test angle assist (roll)")
        self.context_push()
        self.context_collect('STATUSTEXT')
        self.progress("Rolling over to %.0f degrees" % -lim_roll_deg)
        self.set_rc(1, 1000)
        self.wait_roll(-lim_roll_deg, 5)
        self.progress("Killing aileron servo output to force qassist to help")
        self.set_parameter("SERVO1_MIN", 1480)
        self.set_parameter("SERVO1_MAX", 1480)
        self.set_parameter("SERVO1_TRIM", 1480)
        self.progress("Trying to roll over hard the other way")
        self.set_rc(1, 2000)
        self.progress("Waiting for qassist (angle) to kick in")
        self.wait_servo_channel_value(5, 1100, timeout=30, comparator=operator.gt)
        self.wait_statustext('Angle assist', check_context=True)
        self.wait_roll(lim_roll_deg, 5)
        self.context_pop()
        self.set_rc(1, 1500)
        self.progress("Checking qassist stops")
        # we must push RC3 here or the translational drag from the
        # motors keeps our airspeed below AIRSPEED_MIN, and the
        # transition back to pure fixed-wing never completes
        self.set_rc(3, 1800)
        self.wait_servo_channel_value(
            5,
            thr_min_pwm,
            timeout=60,
            comparator=operator.eq,
        )
        self.set_rc(3, 1300)

        self.start_subtest("Test angle assist (pitch-down)")
        self.context_push()
        self.context_collect('STATUSTEXT')
        self.progress("Pitching down to %.0f degrees" % lim_pitch_down_deg)
        self.set_rc(2, 1000)
        self.wait_pitch(lim_pitch_down_deg, accuracy=5)
        self.progress("Killing elevator servo output to force qassist to help")
        self.set_parameters({
            "SERVO2_MIN": 1480,
            "SERVO2_MAX": 1480,
            "SERVO2_TRIM": 1480,
        })
        self.progress("Trying to pitch up hard")
        self.set_rc(2, 2000)
        self.progress("Waiting for qassist (angle) to kick in")
        self.wait_servo_channel_value(5, 1100, timeout=30, comparator=operator.gt)
        self.wait_statustext('Angle assist', check_context=True)
        self.set_rc(2, 1500)
        self.wait_pitch(0, accuracy=5)
        self.context_pop()
        self.progress("Checking qassist stops")
        # we must push RC3 here or the translational drag from the
        # motors keeps our airspeed below AIRSPEED_MIN, and the
        # transition back to pure fixed-wing never completes
        self.set_rc(3, 1800)
        self.wait_servo_channel_value(
            5,
            thr_min_pwm,
            timeout=30,
            comparator=operator.eq,
        )
        self.set_rc(3, 1300)

        self.start_subtest("Test angle assist (pitch-up)")
        self.context_push()
        self.context_collect('STATUSTEXT')
        self.progress("Pitching up to %.0f degrees" % lim_pitch_up_deg)
        self.set_rc(3, 2000)
        self.delay_sim_time(5, reason="pitch angle to build")
        self.change_mode('MANUAL')
        self.context_push()
        self.context_set_speedup(1)
        self.set_rc(2, 1600)
        self.wait_pitch(lim_pitch_up_deg+5, accuracy=5)
        self.context_pop()
        self.progress("Killing elevator servo output to force qassist to help")
        servo2_out = self.get_servo_channel_value(2)
        self.set_parameters({
            "SERVO2_MIN": servo2_out,
            "SERVO2_MAX": servo2_out,
            "SERVO2_TRIM": servo2_out,
        })
        self.change_mode('FBWA')
        self.progress("Trying to pitch down hard")
        self.set_rc(2, 1000)
        self.progress("Waiting for qassist (angle) to kick in")
        self.wait_servo_channel_value(5, 1100, timeout=30, comparator=operator.gt)
        self.wait_statustext('Angle assist', check_context=True)
        self.set_rc(2, 1500)
        self.wait_pitch(0, accuracy=5)
        self.context_pop()
        self.progress("Checking qassist stops")
        # we must push RC3 here or the translational drag from the
        # motors keeps our airspeed below AIRSPEED_MIN, and the
        # transition back to pure fixed-wing never completes
        self.set_rc(3, 1800)
        self.wait_servo_channel_value(
            5,
            thr_min_pwm,
            timeout=30,
            comparator=operator.eq,
        )
        self.set_rc(3, 1300)

        # Test alt assist, climb to 60m and set assist alt to 50m
        self.context_push()
        guided_loc = self.home_position_as_location()
        guided_loc.set_alt_m(60, AltFrame.ABOVE_HOME)
        self.change_mode("GUIDED")
        self.send_do_reposition(guided_loc)
        self.wait_altitude(58, 62, relative=True, timeout=120)
        self.set_parameter("Q_ASSIST_ALT", 50)

        # Try and descent to 40m
        guided_loc.set_alt_m(40, AltFrame.ABOVE_HOME)
        self.send_do_reposition(guided_loc)

        # Expect alt assist to kick in, eg "Alt assist 48.9m"
        self.wait_statustext(r"Alt assist \d*.\d*m", regex=True, timeout=100)

        # Test transition timeout, should switch to QRTL
        self.set_parameter("Q_TRANS_FAIL_ACT", 1)
        self.set_parameter("Q_TRANS_FAIL", 10)
        self.wait_mode("QRTL")

        self.context_pop()

        self.wait_disarmed(timeout=200)
