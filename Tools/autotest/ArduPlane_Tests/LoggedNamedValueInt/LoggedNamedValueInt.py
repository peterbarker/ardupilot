'''
Plane autotest: LoggedNamedValueInt

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class LoggedNamedValueInt(TestGroup):
    @testcase(bucket="PlaneTests1a")
    def LoggedNamedValueInt(self):
        '''ensure that sent named value ints are logged'''
        self.install_example_script_context('simple_loop.lua')
        self.set_parameters({
            'SCR_ENABLE': 1,
            # we never arm, so without this the vehicle is not logging
            # at all: current_onboard_log_filepath() then hands back a
            # log some earlier test left behind, which will never
            # contain our record however long we wait for it.
            'LOG_DISARMED': 1,
        })
        self.reboot_sitl()
        self.wait_ready_to_arm()
        self.wait_statustext('hello, world')
        m = self.assert_received_message_field_values('NAMED_VALUE_INT', {
            "name": "Lua Int",
        })
        dfreader = self.dfreader_for_current_onboard_log()

        m = dfreader.recv_match(type='NVI')
        if m is None:
            raise NotAchievedException("Did not find NVI message")
        self.progress(f"Received NVI with value {m.Value}")
