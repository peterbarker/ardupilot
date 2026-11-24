'''
Copter autotest: NavControllerOutputAltError

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class NavControllerOutputAltError(TestGroup):
    @testcase(bucket="CopterTests2d")
    def NavControllerOutputAltError(self):
        '''check the sign of NAV_CONTROLLER_OUTPUT.alt_error'''
        # alt_error is target altitude minus current altitude, so it
        # is positive when the vehicle is below its target, as sent
        # by Plane, QuadPlane, Sub and PX4
        self.set_parameters({
            "SIM_ENGINE_MUL": 1,
            "SIM_ENGINE_FAIL": 0b1111,  # all four motors
        })
        self.takeoff(20, mode='GUIDED')
        self.delay_sim_time(5, reason="settle in hover")

        self.progress("Weakening all motors so the vehicle sinks below its target")
        self.set_parameter("SIM_ENGINE_MUL", 0.6)
        self.wait_altitude(10, 15, relative=True, timeout=60)
        self.wait_and_maintain_range(
            "alt_error",
            0.5,
            100,
            current_value_getter=lambda: self.assert_receive_message('NAV_CONTROLLER_OUTPUT').alt_error,
            minimum_duration=2,
            timeout=10,
        )

        self.set_parameter("SIM_ENGINE_MUL", 1)
        self.do_RTL()
