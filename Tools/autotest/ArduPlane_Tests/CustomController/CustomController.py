'''
Plane autotest: CustomController

AP_FLAKE8_CLEAN
'''

import vehicle_test_suite

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class CustomController(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def CustomController(self):
        '''Test Custom Controller API'''

        CC_CHANNEL = 6
        # Channel assignments correspond to AP_CustomControl_PID.cpp configuration.
        CUSTOM_CHANNEL = 5
        AIRBRAKE_CHANNEL = 7
        AIRBRAKE_CHANNEL_2 = 10
        PARACHUTE_CHANNEL = 8
        UNDEFINED_CHANNEL = 6
        SERVO_MIN = 1100
        SCALING = 4/5  # Scaling factor between a 1000-2000 input to a 1100-1900 servo.

        self.progress("Configure custom controller parameters")
        self.set_parameters({
            'CP_TYPE': 2,
            'CP_MASK': 65535,
            f'RC{CC_CHANNEL}_OPTION': 109,  # Configure CP switch.
            # Configure an input channel to read flap control separate from the stock flap input.
            f'RC{CUSTOM_CHANNEL}_OPTION': 29,  # A custom input channel (i.e. LANDING_GEAR)
            f'SERVO{CUSTOM_CHANNEL}_FUNCTION': 26,  # Configure a custom output channel (i.e. STEERING)
            f'SERVO{AIRBRAKE_CHANNEL}_FUNCTION': 110,  # Configure airbrake output.
            f'SERVO{AIRBRAKE_CHANNEL_2}_FUNCTION': 110,  # Configure another airbrake output.
            # Configure a parachute output that is driven only by the custom controller.
            f'SERVO{PARACHUTE_CHANNEL}_FUNCTION': 27,
        })
        self.set_rc_from_map({
            CC_CHANNEL: 1000,
            AIRBRAKE_CHANNEL: 1000,
        })
        self.reboot_sitl()
        # Some adjustments to pass the inverted flight test with more margin.
        self.set_parameters({
            # roll
            "CP2_RAT_RLL_P": 0.27,
            "CP2_RAT_RLL_I": 0.225,
            "CP2_RAT_RLL_D": 0.015,
            "CP2_RAT_RLL_FF": 0.213,
            # pitch
            "CP2_RAT_PIT_P": 0.135,
            "CP2_RAT_PIT_I": 0.1,
            "CP2_RAT_PIT_IMAX": 0.9,
            "CP2_RAT_PIT_D": 0.0,
            "CP2_RAT_PIT_FF": 0.536,
        })

        if self.get_parameter("CP_TYPE") != 2 :
            raise NotAchievedException("Custom controller is not switched to PID backend.")

        # check if we can retrieve any param inside PID backend
        self.get_parameter("CP2_RAT_RLL_P")

        # takeoff in GPS mode and perform a standard maneuver: fly straight, then loiter.
        self.takeoff(100)
        self.set_rc(3, 1500)
        self.change_mode("CRUISE")
        self.delay_sim_time(10, "Let the plane fly straight and level.")
        self.change_mode("LOITER")
        self.delay_sim_time(30, "Let the plane settle on the loiter.")
        # Return to level flight.
        self.change_mode("CRUISE")
        self.delay_sim_time(10, "Let the plane fly straight and level.")

        self.context_push()
        self.context_collect('STATUSTEXT')

        # switch custom controller on
        self.set_rc(CC_CHANNEL, 2000)
        self.wait_statustext("Custom controller is ON", check_context=True)
        self.delay_sim_time(10, "Give some time to the custom controller to establish level flight.")
        self.change_mode("LOITER")

        # wait 30 seconds to see if the custom controller destabilize the aircraft
        current_alt = self.get_altitude(relative=True)
        self.wait_altitude(current_alt-10, current_alt+10, relative=True, minimum_duration=30, timeout=40)

        # ensure we can fly inverted
        self.run_auxfunc(43, 2)  # 43 == inverted flight
        self.wait_altitude(current_alt-20, current_alt+10, relative=True, minimum_duration=30, timeout=40)
        self.run_auxfunc(43, 0)

        # Ensure we can manipulate the outputs in various ways.

        self.set_rc(CUSTOM_CHANNEL, 1800)
        w = vehicle_test_suite.WaitAndMaintainServoChannelValue(
            self,
            CUSTOM_CHANNEL,
            1500 + (1800-1500)*SCALING,  # Ensure we can address outputs by function and drive them with unit inputs.
            minimum_duration=1,
        )
        w.run()
        self.set_rc(CUSTOM_CHANNEL, 1500)

        self.set_rc(1, 1800)
        self.wait_servo_channel_value(UNDEFINED_CHANNEL, 1800)  # Ensure we can control unused channels with pwm values.
        self.set_rc(1, 1500)

        self.assert_servo_channel_value(AIRBRAKE_CHANNEL, SERVO_MIN)  # Ensure that the function output is at minimum.
        self.assert_servo_channel_value(AIRBRAKE_CHANNEL_2, 1000)  # Direct PWM writes don't respect min/max.
        self.assert_servo_channel_value(PARACHUTE_CHANNEL, 1000)  # Direct PWM writes don't respect min/max.
        self.set_rc(AIRBRAKE_CHANNEL, 1800)
        # Ensure we don't override servos.cpp by default. We haven't configured an airbrake input.
        # servos.cpp overrides us. We expect zero output here.
        self.wait_servo_channel_value(AIRBRAKE_CHANNEL, SERVO_MIN)
        self.wait_servo_channel_value(AIRBRAKE_CHANNEL_2, 1800)  # Ensure channel overrides work.
        self.wait_servo_channel_value(PARACHUTE_CHANNEL, 1800)  # Ensure a channel can be controlled by function addressing.
        self.set_rc(AIRBRAKE_CHANNEL, 1000)

        # Ensure output masking works.
        self.set_parameter("CP_MASK", 65407)  # The custom PID controller puts the 2nd airbrake output on bit7.
        self.set_rc(AIRBRAKE_CHANNEL, 1800)
        # Ensure masking works and the airbrake is set by its non-custom source.
        self.wait_servo_channel_value(PARACHUTE_CHANNEL, 1800)  # Ensure this channel is still active.
        self.wait_servo_channel_value(AIRBRAKE_CHANNEL_2, SERVO_MIN)
        self.set_rc(AIRBRAKE_CHANNEL, 1000)

        # switch custom controller off
        self.set_rc(CC_CHANNEL, 1000)
        self.wait_statustext("Custom controller is OFF", check_context=True)

        self.context_pop()
        self.fly_home_land_and_disarm()
