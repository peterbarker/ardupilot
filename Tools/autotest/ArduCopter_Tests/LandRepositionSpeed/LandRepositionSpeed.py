'''
Copter autotest: LandRepositionSpeed

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class LandRepositionSpeed(TestGroup):
    @testcase(bucket="CopterTests2b")
    def LandRepositionSpeed(self):
        '''check LAND reposition speed is independent of heading, with and without SIMPLE'''

        # reboot so the simple heading is captured at the 270 degree start heading
        self.reboot_sitl()

        self.set_parameters({
            "FLTMODE_CH": 5,
            "FLTMODE1": 5,  # Loiter
            "FLTMODE2": 9,  # Land
            "LAND_SPD_HIGH_MS": 0.5,
        })

        # reposition speed is limited to half the waypoint acceleration
        want_speed = 0.5 * self.get_parameter("WP_ACC")

        self.takeoff(40, mode="LOITER")
        self.wait_heading(270, 5, timeout=60)

        # yaw, simple, roll pwm, pitch pwm
        cases = [
            (270, False, 1500, 1000),
            (315, False, 1500, 1000),
            (315, False, 2000, 1000),
            (315, True, 1500, 1000),
        ]
        for (yaw_angle, simple, roll_pwm, pitch_pwm) in cases:
            self.progress("yaw=%u simple=%s roll=%u pitch=%u" % (yaw_angle, simple, roll_pwm, pitch_pwm))

            self.set_rc(4, 1560)
            self.wait_heading(yaw_angle, timeout=60)
            self.set_rc(4, 1500)

            # SIMPLE is applied to FLTMODE2 when the mode switch moves to it
            self.set_parameter("SIMPLE", 2 if simple else 0)
            self.set_rc(5, 1298)
            self.wait_mode("LAND")

            self.set_rc_from_map({
                1: roll_pwm,
                2: pitch_pwm,
            })
            self.wait_groundspeed(want_speed - 0.15, want_speed + 0.15, minimum_duration=3, timeout=20)
            self.set_rc_from_map({
                1: 1500,
                2: 1500,
            })

            self.set_rc(5, 1165)
            self.wait_mode("LOITER")
            self.wait_groundspeed(0, 0.3, timeout=30)

        self.do_RTL(timeout=500)
