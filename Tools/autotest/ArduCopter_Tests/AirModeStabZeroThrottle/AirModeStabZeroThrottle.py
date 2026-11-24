'''
Copter autotest: AirModeStabZeroThrottle

AP_FLAKE8_CLEAN
'''

import math

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class AirModeStabZeroThrottle(TestGroup):
    @testcase(bucket="CopterTests2b")
    def AirModeStabZeroThrottle(self):
        '''test air-mode gives attitude control with the throttle stick at zero'''

        def wait_throttle_output(minimum, maximum, **kwargs):
            """wait for the throttle the autopilot is asking the motors for"""
            self.wait_and_maintain_range(
                value_name="ThrottleOutput",
                minimum=minimum,
                maximum=maximum,
                current_value_getter=lambda: self.assert_receive_message('VFR_HUD').throttle,
                **kwargs)

        def assert_motor_outputs(minimum, maximum):
            """check all four motor outputs are within the given PWM range"""
            m = self.assert_receive_message('SERVO_OUTPUT_RAW')
            values = [m.servo1_raw, m.servo2_raw, m.servo3_raw, m.servo4_raw]
            self.progress("motor outputs: %s" % str(values))
            for (i, value) in enumerate(values):
                if value < minimum or value > maximum:
                    raise NotAchievedException(
                        "Motor %u output %u not in range %u-%u" %
                        (i+1, value, minimum, maximum))

        def airmode_flight(spin_arm_pwm):
            """fly the scenario from the air-mode bug report: hover in
            STABILIZE, check the pilot has roll control, then drop the
            throttle to zero and check the vehicle still holds its attitude
            against a disturbance all the way down"""

            # take off in GUIDED, so that getting airborne does not depend on
            # where the throttle stick is:
            self.change_mode('GUIDED')
            self.user_takeoff(alt_min=120, timeout=180)

            self.progress("hover in STABILIZE with the throttle stick at mid")
            self.change_mode('STABILIZE')
            self.set_rc(3, 1500)
            wait_throttle_output(20, 80, minimum_duration=1)
            assert_motor_outputs(1300, 1900)

            self.progress("check the pilot has roll control")
            self.set_rc(1, 1000)
            self.wait_roll(-30, 5, timeout=10)
            self.set_rc(1, 1500)
            self.wait_roll(0, 5, timeout=10)

            self.progress("drop the throttle to zero; air-mode should keep the "
                          "vehicle stabilised on the way down")
            self.zero_throttle()
            # the pilot is asking for nothing; anything the motors are given
            # from here is air-mode keeping the vehicle stabilised:
            wait_throttle_output(0, 1, minimum_duration=1)

            # a sustained roll torque the attitude controller has to fight;
            # with air-mode working the vehicle holds attitude, without it the
            # vehicle tumbles:
            self.set_parameters({
                "SIM_TWIST_X": 40,  # rad/s/s
                "SIM_TWIST_TIME": 3000,  # ms
            })

            max_roll = 0
            tstart = self.get_sim_time()
            while self.get_sim_time_cached() - tstart < 3:
                m = self.assert_receive_message('GLOBAL_POSITION_INT')
                if m.relative_alt * 0.001 < 40:
                    # stop well before we reach the ground
                    break
                assert_motor_outputs(spin_arm_pwm + 10, 2000)
                m = self.assert_receive_message('ATTITUDE')
                roll = abs(math.degrees(m.roll))
                max_roll = max(max_roll, roll)
                if roll > 20:
                    raise NotAchievedException(
                        "Vehicle not stabilised at zero throttle (roll=%.1fdeg)" % roll)
            self.progress("maximum roll excursion %.1fdeg" % max_roll)
            if max_roll < 1:
                # the vehicle did not feel the disturbance, so we have not
                # actually tested anything:
                raise NotAchievedException("Disturbance was not applied")

            self.set_parameters({
                "SIM_TWIST_X": 0,
                "SIM_TWIST_TIME": 0,
            })
            self.disarm_vehicle(force=True)
            self.zero_throttle()
            self.reboot_sitl()

        def arm_with_aux_switch():
            """arm normally, then turn air-mode on with the AIRMODE aux switch"""
            self.set_rc_from_map({7: 1000, 8: 1000})
            self.wait_ready_to_arm()
            self.zero_throttle()
            self.arm_vehicle()
            self.set_rc(8, 2000)

        def arm_with_arming_switch():
            """arm with the ARMDISARM_AIRMODE switch, which implies air-mode"""
            self.set_rc_from_map({7: 1000, 8: 1000})
            self.wait_ready_to_arm()
            self.zero_throttle()
            self.arm_motors_with_switch(7)

        # ATC_THR_MIX_MAN defaults to the same value as ATC_THR_MIX_MIN, which
        # leaves the attitude controller with almost no authority at zero
        # throttle; air-mode users raise it:
        self.set_parameters({
            "ATC_THR_MIX_MAN": 0.5,
            "RC7_OPTION": 154,  # ARMDISARM_AIRMODE
            "RC8_OPTION": 84,   # AIRMODE
        })
        # motors idle at MOT_SPIN_ARM when the vehicle thinks it is not
        # flying; the motor PWM range is the default 1000-2000:
        spin_arm_pwm = 1000 + 1000 * self.get_parameter("MOT_SPIN_ARM")

        for (description, arm_method) in [
                ("the AIRMODE aux switch", arm_with_aux_switch),
                ("arming with the ARMDISARM_AIRMODE switch", arm_with_arming_switch),
        ]:
            self.start_subtest("stabilisation at zero throttle with %s" % description)
            arm_method()
            airmode_flight(spin_arm_pwm)
