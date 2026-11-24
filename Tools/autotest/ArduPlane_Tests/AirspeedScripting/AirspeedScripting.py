'''
Plane autotest: AirspeedScripting

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class AirspeedScripting(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def AirspeedScripting(self):
        '''test airspeed data supplied by a lua script'''
        self.set_parameters({
            'SCR_ENABLE': 1,
            'ARSPD2_TYPE': 20,  # Scripting
            'ARSPD2_USE': 0,
        })
        self.install_test_script_context('airspeed-scripting.lua')
        self.reboot_sitl()

        # Script sets 30m/s airspeed and 25 deg temperature
        self.wait_message_field_values('AIRSPEED', {
            'airspeed': 30.0,
            'temperature': 2500,
        }, instance=1, epsilon=0.5, minimum_duration=2, timeout=30)
