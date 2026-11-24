'''
Plane autotest: LoggedNamedValueString

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class LoggedNamedValueString(TestGroup):
    @testcase(bucket="PlaneTests1c")
    def LoggedNamedValueString(self):
        '''ensure that sent named value strings are logged'''
        self.install_example_script_context('simple_named_string.lua')
        self.set_parameters({
            'SCR_ENABLE': 1,
            # we never arm; see LoggedNamedValueFloat
            'LOG_DISARMED': 1,
        })
        self.reboot_sitl()
        self.wait_ready_to_arm()
        m = self.assert_received_message_field_values('NAMED_VALUE_STRING', {
            "name": "Lua String",
            "value": "Lua String Value",
        })
        dfreader = self.dfreader_for_current_onboard_log()

        m = dfreader.recv_match(type='NVS')
        if m is None:
            raise NotAchievedException("Did not find NVS message")
        self.progress(f"Received NVS with value {m.Value}")
        if m.Name != 'Lua String':
            raise NotAchievedException("Unexpected name in NVS")
        if m.Value != 'Lua String Value':
            raise NotAchievedException("Unexpected value in NVS")
