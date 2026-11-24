'''
Sub autotest: AltitudeHold

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase

from ardusub import Joystick


class AltitudeHold(TestGroup):
    @testcase(bucket="Sub")
    def AltitudeHold(self):
        """Test ALT_HOLD mode"""
        # a context so the simulated buoyancy set below is put back:
        # MotorThrustHoverParameterIgnore calls this twice, and the second
        # dive starts at the surface, where the extra lift stops it
        self.context_push()
        self.dive(-5, mode='ALT_HOLD')
        self.watch_altitude_maintained()

        self.set_rc(Joystick.Throttle, 1000)
        self.wait_altitude(altitude_min=-20, altitude_max=-19)
        self.set_rc(Joystick.Throttle, 1500)

        # let the vehicle settle (momentum / stopping point shenanigans....)
        self.delay_sim_time(1, reason="vehicle to settle")

        self.watch_altitude_maintained()

        self.set_rc(Joystick.Throttle, 1900)
        self.wait_altitude(altitude_min=-14, altitude_max=-13)
        self.set_rc(Joystick.Throttle, 1500)

        # let the vehicle settle (momentum / stopping point shenanigans....)
        self.delay_sim_time(1, reason="vehicle to settle")

        self.watch_altitude_maintained()

        self.set_rc(Joystick.Throttle, 1900)
        self.wait_altitude(altitude_min=-5, altitude_max=-4)
        self.set_rc(Joystick.Throttle, 1500)

        # let the vehicle settle (momentum / stopping point shenanigans....)
        self.delay_sim_time(1, reason="vehicle to settle")
        self.watch_altitude_maintained()

        # Make sure the code can handle buoyancy changes
        self.set_parameter("SIM_BUOYANCY", 10)
        self.watch_altitude_maintained()
        self.set_parameter("SIM_BUOYANCY", -10)
        self.watch_altitude_maintained()

        # Make sure that the ROV will dive with a small input down even if there is a 10N buoyancy force upwards
        self.set_parameter("SIM_BUOYANCY", 10)
        self.set_rc(Joystick.Throttle, 1350)
        self.wait_altitude(altitude_min=-6, altitude_max=-5.5)

        self.set_rc(Joystick.Throttle, 1500)
        self.watch_altitude_maintained()
        self.disarm_vehicle()
        self.context_pop()
