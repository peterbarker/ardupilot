'''
Copter autotest: MotorFail

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class MotorFail(TestGroup):
    @testcase(bucket="CopterTests2c")
    def MotorFail(self, ):
        """Test flight with reduced motor efficiency"""
        # we only expect an octocopter to survive ATM:
        self.MotorFail_test_frame('octa', 8, frame_class=3)
        # self.MotorFail_test_frame('hexa', 6, frame_class=2)
        # self.MotorFail_test_frame('y6', 6, frame_class=5)

    def MotorFail_test_frame(self, model, servo_count, frame_class, fail_servo=0, fail_mul=0.0, holdtime=30):
        self.set_parameters({
            'FRAME_CLASS': frame_class,
        })
        self.customise_SITL_commandline([], model=model)

        self.takeoff(25, mode="LOITER")

        # Get initial values
        start_hud = self.assert_receive_message('VFR_HUD')
        start_attitude = self.assert_receive_message('ATTITUDE')

        hover_time = 5
        tstart = self.get_sim_time()
        int_error_alt = 0
        int_error_yaw_rate = 0
        int_error_yaw = 0
        self.progress("Hovering for %u seconds" % hover_time)
        failed = False
        while True:
            now = self.get_sim_time_cached()
            if now - tstart > holdtime + hover_time:
                break

            servo = self.assert_receive_message('SERVO_OUTPUT_RAW')
            hud = self.assert_receive_message('VFR_HUD')
            attitude = self.assert_receive_message('ATTITUDE')

            if not failed and now - tstart > hover_time:
                self.progress("Killing motor %u (%u%%)" %
                              (fail_servo+1, fail_mul))
                self.set_parameters({
                    "SIM_ENGINE_MUL": fail_mul,
                    "SIM_ENGINE_FAIL": 1 << fail_servo,
                })
                failed = True

            if failed:
                self.progress("Hold Time: %f/%f" % (now-tstart, holdtime))

            servo_pwm = [
                servo.servo1_raw,
                servo.servo2_raw,
                servo.servo3_raw,
                servo.servo4_raw,
                servo.servo5_raw,
                servo.servo6_raw,
                servo.servo7_raw,
                servo.servo8_raw,
            ]

            self.progress("PWM output per motor")
            for i, pwm in enumerate(servo_pwm[0:servo_count]):
                if pwm > 1900:
                    state = "oversaturated"
                elif pwm < 1200:
                    state = "undersaturated"
                else:
                    state = "OK"

                if failed and i == fail_servo:
                    state += " (failed)"

                self.progress("servo %u [pwm=%u] [%s]" % (i+1, pwm, state))

            alt_delta = hud.alt - start_hud.alt
            yawrate_delta = attitude.yawspeed - start_attitude.yawspeed
            yaw_delta = attitude.yaw - start_attitude.yaw

            self.progress("Alt=%fm (delta=%fm)" % (hud.alt, alt_delta))
            self.progress("Yaw rate=%f (delta=%f) (rad/s)" %
                          (attitude.yawspeed, yawrate_delta))
            self.progress("Yaw=%f (delta=%f) (deg)" %
                          (attitude.yaw, yaw_delta))

            dt = self.get_sim_time() - now
            int_error_alt += abs(alt_delta/dt)
            int_error_yaw_rate += abs(yawrate_delta/dt)
            int_error_yaw += abs(yaw_delta/dt)
            self.progress("## Error Integration ##")
            self.progress("  Altitude: %fm" % int_error_alt)
            self.progress("  Yaw rate: %f rad/s" % int_error_yaw_rate)
            self.progress("  Yaw: %f deg" % int_error_yaw)
            self.progress("----")

            if int_error_yaw > 5:
                raise NotAchievedException("Vehicle is spinning")

            if alt_delta < -20:
                raise NotAchievedException("Vehicle is descending")

        self.progress("Fixing motors")
        self.set_parameter("SIM_ENGINE_FAIL", 0)

        self.do_RTL()
