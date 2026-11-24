'''
Plane autotest: TestAutoSpeedFlaps

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class TestAutoSpeedFlaps(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def TestAutoSpeedFlaps(self):
        """Test auto flap deployment based on airspeed thresholds."""
        servo_ch = 5
        servo_ch_min = 1200
        servo_ch_max = 1800

        flap_1_speed = 18   # m/s
        flap_2_speed = 12   # m/s
        flap_1_percent = 30
        flap_2_percent = 60

        def flap_pct_to_pwm(pct):
            return int(servo_ch_min + (pct / 100.0) * (servo_ch_max - servo_ch_min))

        flap_1_pwm = flap_pct_to_pwm(flap_1_percent)   # 1380
        flap_2_pwm = flap_pct_to_pwm(flap_2_percent)   # 1560
        pwm_epsilon = 20

        self.set_parameters({
            "SERVO%u_FUNCTION" % servo_ch: 3,     # flapsauto
            "SERVO%u_MIN" % servo_ch: servo_ch_min,
            "SERVO%u_MAX" % servo_ch: servo_ch_max,
            "SERVO%u_TRIM" % servo_ch: servo_ch_min,
            "FLAP_1_SPEED": flap_1_speed,
            "FLAP_2_SPEED": flap_2_speed,
            "FLAP_1_PERCNT": flap_1_percent,
            "FLAP_2_PERCNT": flap_2_percent,
            "FLAP_SLEWRATE": 100,
            "TKOFF_FLAP_PCNT": 0,
            "AIRSPEED_CRUISE": 22,
            "AIRSPEED_MAX": 30,
            "ARSPD_USE": 1,
        })

        self.takeoff(alt=100, mode="TAKEOFF", timeout=120)
        self.set_rc(3, 1500)
        self.change_mode("GUIDED")
        self.delay_sim_time(5, reason="establish guided flight")

        self.start_subtest("Target-airspeed-based auto flap deployment in GUIDED mode")

        self.progress("No flaps when target above FLAP_1_SPEED")
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            p1=0,   # airspeed type
            p2=22,  # above FLAP_1_SPEED=18
            p3=-1,
        )
        self.wait_servo_channel_value(servo_ch, servo_ch_min, epsilon=pwm_epsilon, timeout=10)

        self.progress("FLAP_1 deploys when target drops below FLAP_1_SPEED")
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            p1=0,
            p2=15,  # between FLAP_2_SPEED=12 and FLAP_1_SPEED=18
            p3=-1,
        )
        self.wait_servo_channel_value(servo_ch, flap_1_pwm, epsilon=pwm_epsilon, timeout=10)

        self.progress("FLAP_2 deploys when target drops below FLAP_2_SPEED")
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            p1=0,
            p2=10,  # below FLAP_2_SPEED=12
            p3=-1,
        )
        self.wait_servo_channel_value(servo_ch, flap_2_pwm, epsilon=pwm_epsilon, timeout=10)

        self.progress("Flaps retract when target rises above FLAP_1_SPEED")
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            p1=0,
            p2=22,
            p3=-1,
        )
        self.wait_servo_channel_value(servo_ch, servo_ch_min, epsilon=pwm_epsilon, timeout=10)

        self.start_subtest("FLAP_ACTUAL_SPEED deploys flaps in non-auto modes using measured airspeed")
        # Set FLAP_1_SPEED above the cruise airspeed so measured speed will be below threshold
        self.set_parameter("FLAP_1_SPEED", 26)  # above typical cruise of ~22 m/s
        self.set_parameter("FLIGHT_OPTIONS", 1 << 15)  # enable FLAP_ACTUAL_SPEED
        self.change_mode("FBWA")
        # cruise throttle, not full throttle: at 1700 the aircraft settles around
        # 26.6m/s, *above* the FLAP_1_SPEED we just set, so the flaps deploy on the
        # way up through 26 and retract again a few seconds later.  Both assertions
        # below then only hold during that transient, and which side of the bound
        # the first sample lands on decides the run.
        self.set_rc(3, 1500)
        # a settled airspeed below the threshold, not one passing through it:
        self.wait_airspeed(18, 24, timeout=30, minimum_duration=5)
        self.wait_servo_channel_value(servo_ch, flap_1_pwm, epsilon=pwm_epsilon, timeout=15)

        self.progress("Flaps retract when FLAP_ACTUAL_SPEED option is disabled")
        self.set_parameter("FLIGHT_OPTIONS", 0)
        self.wait_servo_channel_value(servo_ch, servo_ch_min, epsilon=pwm_epsilon, timeout=10)

        self.fly_home_land_and_disarm(timeout=180)
