'''
Plane autotest: TestRCRelay

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException
from vehicle_test_suite import PreconditionFailedException

from test_discovery import TestGroup
from test_discovery import testcase


class TestRCRelay(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def TestRCRelay(self):
        '''Test Relay RC Channel Option'''
        self.set_parameters({
            "RELAY1_FUNCTION": 1, # Enable relay as a standard relay pin
            "RC12_OPTION": 28 # Relay On/Off
        })
        self.set_rc(12, 1000)
        self.reboot_sitl() # needed for RC12_OPTION and RELAY1_FUNCTION to take effect

        off = self.get_parameter("SIM_PIN_MASK")
        if off:
            raise PreconditionFailedException("SIM_MASK_PIN off")

        # allow time for the RC library to register initial value:
        self.delay_sim_time(1, reason="RC library to register initial value")

        self.set_rc(12, 2000)
        self.wait_heartbeat()
        self.wait_heartbeat()

        on = self.get_parameter("SIM_PIN_MASK")
        if not on:
            raise NotAchievedException("SIM_PIN_MASK doesn't reflect ON")
        self.set_rc(12, 1000)
        self.wait_heartbeat()
        self.wait_heartbeat()
        off = self.get_parameter("SIM_PIN_MASK")
        if off:
            raise NotAchievedException("SIM_PIN_MASK doesn't reflect OFF")
